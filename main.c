#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <rdma/ib_verbs.h>

#define DRIVER_AUTHOR "Yuval Shaia <yuval.shaia@oracle.com>"
#define DRIVER_DESC   "Pingpong test"
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

#define MAX_DEVS 16

int qp_type = IB_QPT_RC;
module_param_named(qp_type, qp_type, int, 0444);
MODULE_PARM_DESC(qp_type, "QP Type");

struct comm {
	struct ib_device *dev;
	struct ib_pd *pd;
	struct ib_qp *qp;
	bool qp_initialized;
	u8 sport;
	u64 dlid;
	u32 dqpn;
	struct ib_mr *mr;
	int buf_sz;
	char *send_buf;
	u64 send_buf_dma_addr;
	char *recv_buf;
	u64 recv_buf_dma_addr;
};

int comm_idx = -1;
struct comm comm_array[MAX_DEVS];

static struct comm *find_comm_obj(struct device *d)
{
	int i;

	for (i = 0; i <= comm_idx; i++)
		if (comm_array[i].dev->dma_device == d)
			return &comm_array[i];

	return NULL;
}

static int init_qp_state(struct ib_qp *qp, bool rts, int port, u16 dlid,
			 u32 dqpn)
{
	int qp_attr_mask = 0;
	struct ib_qp_attr qp_attr;
	int rc = -1;
	static u32 psn = 1;

	memset(&qp_attr, 0, sizeof(struct ib_qp_attr));

	/* RESET */
	qp_attr_mask |= IB_QP_STATE;
	qp_attr.qp_state = IB_QPS_RESET;
	rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
	if (rc)
		goto err;

	/* INIT */
	memset(&qp_attr, 0, sizeof(struct ib_qp_attr));
	qp_attr_mask = 0;
	qp_attr_mask |= IB_QP_STATE;
	qp_attr.qp_state = IB_QPS_INIT;
	qp_attr_mask |= IB_QP_PKEY_INDEX;
	qp_attr.pkey_index = 0;
	qp_attr_mask |= IB_QP_PORT;
	qp_attr.port_num = port;
	qp_attr_mask |= IB_QP_ACCESS_FLAGS;
	qp_attr.qp_access_flags = IB_ACCESS_REMOTE_WRITE |
				  IB_ACCESS_REMOTE_READ;
	rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
	if (rc)
		goto err;

	/* RTR */
	qp_attr_mask = 0;
	qp_attr_mask |= IB_QP_STATE;
	qp_attr.qp_state = IB_QPS_RTR;
	qp_attr_mask |= IB_QP_AV;
	qp_attr.ah_attr.port_num = port;
	qp_attr.ah_attr.sl = 0;
	qp_attr.ah_attr.ah_flags = 0;
	qp_attr.ah_attr.dlid = dlid;
	qp_attr.ah_attr.static_rate = 2;
	qp_attr.ah_attr.src_path_bits = 0;
	qp_attr_mask |= IB_QP_PATH_MTU;
	qp_attr.path_mtu = IB_MTU_256;
	qp_attr_mask |= IB_QP_RQ_PSN;
	qp_attr.rq_psn = psn;
	qp_attr_mask |= IB_QP_MAX_DEST_RD_ATOMIC;
	qp_attr.max_dest_rd_atomic = 1;
	qp_attr_mask |= IB_QP_DEST_QPN;
	qp_attr.dest_qp_num = dqpn;
	qp_attr_mask |= IB_QP_MIN_RNR_TIMER;
	qp_attr.min_rnr_timer = 0;
	rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
	if (rc)
		goto err;

	/* RTS */
	if (rts) {
		qp_attr_mask = 0;
		qp_attr_mask |= IB_QP_STATE;
		qp_attr.qp_state = IB_QPS_RTS;
		qp_attr_mask |= IB_QP_SQ_PSN;
		qp_attr.sq_psn = psn;
		qp_attr_mask |= IB_QP_TIMEOUT;
		qp_attr.timeout = 0x4;
		qp_attr_mask |= IB_QP_RETRY_CNT;
		qp_attr.retry_cnt = 0;
		qp_attr_mask |= IB_QP_RNR_RETRY;
		qp_attr.rnr_retry = 0;
		qp_attr_mask |= IB_QP_MAX_QP_RD_ATOMIC;
		qp_attr.max_rd_atomic = 1;
		rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
		if (rc)
			goto err;
	}

	/* increase the PSN every time we do modify QP */
	psn += 100000;

	return 0;
err:
	if (rc)
		pr_err("Fail to switch to QP state %d, rc=%d\n",
		       qp_attr.qp_state, rc);
	return rc;
}

static void post_send(struct comm *comm, const char *buf)
{
	struct ib_send_wr wr, *bad_wr;
	struct ib_sge sge;
	int rc;
	struct ib_wc wc;
	int poll_cq_times = 10000;

	comm->send_buf = kzalloc(comm->buf_sz, GFP_KERNEL);
	strncpy(comm->send_buf, buf, comm->buf_sz);

	comm->send_buf_dma_addr = ib_dma_map_single(comm->dev, comm->send_buf,
						    comm->buf_sz,
						    DMA_TO_DEVICE);
	if (ib_dma_mapping_error(comm->dev, comm->send_buf_dma_addr)) {
		pr_err("Fail to map to DMA address\n");
		goto out_kfree;
	}
	sge.addr = comm->send_buf_dma_addr;
	sge.length = comm->buf_sz;
	sge.lkey = comm->mr->lkey;

	wr.next = NULL;
	wr.wr_id = (unsigned long)&comm->send_buf;
	wr.opcode = IB_WR_SEND;
	wr.send_flags = IB_SEND_SIGNALED;
	wr.sg_list = &sge;
	wr.num_sge = 1;

	rc = ib_post_send(comm->qp, &wr, &bad_wr);
	if (rc) {
		pr_err("ib_post_send returned %d\n", rc);
		goto out_dma_unmap;;
	}

	rc = 0;
	while (!rc && poll_cq_times--)
		rc = ib_poll_cq(comm->qp->send_cq, 1, &wc);
	if (rc != 1)
		pr_err("send ib_poll_cq returned %d\n", rc);
	if (wc.status != IB_WC_SUCCESS) {
		pr_err("wc.status=%d\n", wc.status);
		pr_err("wc.vendor_err=0x%x\n", wc.vendor_err);
	}

out_dma_unmap:
	ib_dma_unmap_single(comm->dev, comm->send_buf_dma_addr, comm->buf_sz,
			    DMA_TO_DEVICE);

out_kfree:
	kfree(comm->send_buf);
	comm->send_buf = NULL;
}

static void post_recv(struct comm *comm)
{
	struct ib_recv_wr wr, *bad_wr;
	struct ib_sge sge;
	int rc;

	comm->recv_buf = (char *)kzalloc(comm->buf_sz, GFP_KERNEL);
	strncpy(comm->recv_buf, "INIT_DATA", comm->buf_sz);

	comm->recv_buf_dma_addr = ib_dma_map_single(comm->dev, comm->recv_buf,
						    comm->buf_sz,
						    DMA_FROM_DEVICE);
	if (ib_dma_mapping_error(comm->dev, comm->recv_buf_dma_addr)) {
		pr_err("Fail to map to DMA address\n");
		kfree(comm->recv_buf);
		comm->recv_buf = NULL;
	}
	sge.addr = comm->recv_buf_dma_addr;
	sge.length = comm->buf_sz;
	sge.lkey = comm->mr->lkey;

	wr.next = NULL;
	wr.wr_id = (unsigned long)&comm->recv_buf;
	wr.sg_list = &sge;
	wr.num_sge = 1;

	rc = ib_post_recv(comm->qp, &wr, &bad_wr);
	if (rc) {
		ib_dma_unmap_single(comm->dev, sge.addr, sge.length,
				    DMA_TO_DEVICE);
		pr_err("ib_post_recv returned %d\n", rc);
	}
}

static ssize_t show_buf_sz(struct device *d, struct device_attribute *attr,
			   char *buf)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	return sprintf(buf, "%d\n", comm->buf_sz);
}

static ssize_t store_buf_sz(struct device *d, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	if (kstrtol(buf, 0, (long *)&comm->buf_sz))
		pr_warn("Invalid format for buf_sz (%s)\n", buf);

	return count;
}

static DEVICE_ATTR(buf_sz, S_IWUSR | S_IRUGO, show_buf_sz, store_buf_sz);

static ssize_t show_sport(struct device *d, struct device_attribute *attr,
			  char *buf)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	return sprintf(buf, "%d\n", comm->sport);
}

static ssize_t store_sport(struct device *d, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	if (kstrtol(buf, 0, (long *)&comm->sport))
		pr_warn("Invalid format for sport (%s)\n", buf);

	comm->qp_initialized = false;
	return count;
}

static DEVICE_ATTR(sport, S_IWUSR | S_IRUGO, show_sport, store_sport);

static ssize_t show_dlid(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	return sprintf(buf, "%ld\n", (long)comm->dlid);
}
static ssize_t store_dlid(struct device *d, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	if (kstrtol(buf, 0, (long *)&comm->dlid))
		pr_warn("Invalid format for dlid (%s)\n", buf);

	comm->qp_initialized = false;
	return count;
}

static DEVICE_ATTR(dlid, S_IWUSR | S_IRUGO, show_dlid, store_dlid);

static ssize_t show_dqpn(struct device *d, struct device_attribute *attr,
			 char *buf)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	return sprintf(buf, "%d\n", comm->dqpn);
}

static ssize_t store_dqpn(struct device *d, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	if (kstrtol(buf, 0, (long *)&comm->dqpn))
		pr_warn("Invalid format for dqpn (%s)\n", buf);

	return count;
}

static DEVICE_ATTR(dqpn, S_IWUSR | S_IRUGO, show_dqpn, store_dqpn);

static int verify_comm_param(struct comm *comm)
{
	struct ib_port_attr port_attr;
	int rc;

	/* Do local test if dlid was not specified */
	if (!comm->sport)
		comm->sport = 1;

	if (!comm->dlid) {
		rc = ib_query_port(comm->dev, comm->sport, &port_attr);
		if (rc) {
			pr_err("Fail to query port %d (err=%d)\n",
			       comm->sport, rc);
			return rc;
		}
		comm->dlid = port_attr.lid;
	}

	return 0;
}

static ssize_t send(struct device *d, struct device_attribute *attr,
		    const char *buf, size_t count)
{
	int rc;
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	rc = verify_comm_param(comm);
	if (rc) {
		pr_err("Fail to set communicatin parameters\n");
		return rc;
	}

	if (!comm->qp_initialized) {
		rc = init_qp_state(comm->qp, true, comm->sport, comm->dlid,
				   comm->dqpn);
		if (rc) {
			pr_err("Fail to change QP state to send\n");
			return rc;
		}
		comm->qp_initialized = true;
	}

	post_send(comm, buf);

	return count;
}

static DEVICE_ATTR(send, S_IWUSR, NULL, send);

static ssize_t show_recv_buf(struct device *d, struct device_attribute *attr,
			     char *buf)
{
	int rc;
	struct comm *comm = find_comm_obj(d);
	struct ib_wc wc;
	int poll_cq_times = 10000;

	if (!comm)
		return 0;

	if (comm->recv_buf == NULL)
		return 0;

	rc = 0;
	while (!rc && poll_cq_times--)
		rc = ib_poll_cq(comm->qp->recv_cq, 1, &wc);
	if (rc != 1)
		pr_err("recv ib_poll_cq returned %d\n", rc);
	ib_dma_unmap_single(comm->dev, comm->recv_buf_dma_addr, comm->buf_sz,
			    DMA_TO_DEVICE);

	comm->recv_buf[comm->buf_sz - 1] = 0;
	rc = sprintf(buf, "%s\n", comm->recv_buf);

	kfree(comm->recv_buf);
	comm->recv_buf = NULL;

	return rc;
}

static ssize_t recv(struct device *d, struct device_attribute *attr,
		    const char *buf, size_t count)
{
	int rc;
	struct comm *comm = find_comm_obj(d);

	if (!comm)
		return 0;

	rc = verify_comm_param(comm);
	if (rc) {
		pr_err("Fail to set communication parameters\n");
		return rc;
	}

	if (!comm->qp_initialized) {
		rc = init_qp_state(comm->qp, true, comm->sport, comm->dlid,
				   comm->dqpn);
		if (rc) {
			pr_err("Fail to change QP state to recv\n");
			return rc;
		}
		comm->qp_initialized = true;
	}

	post_recv(comm);

	return count;
}

static DEVICE_ATTR(recv, S_IWUSR | S_IRUGO, show_recv_buf, recv);

static void add_one(struct ib_device *device)
{
	struct ib_cq *scq = 0, *rcq = 0;
	struct ib_qp_init_attr qp_init_attr;
	struct comm *comm;
	struct ib_cq_init_attr cq_attr;

	pr_info("Pingpong.add_one: %s\n", device->name);

	comm_idx++;
	comm = &comm_array[comm_idx];

	memset(comm, 0, sizeof(struct comm));

	comm->buf_sz = 1024;

	/* PD */
	comm->pd = ib_alloc_pd(device);
	if (!comm->pd)
		return;

	/* CQs */
	memset(&cq_attr, 0, sizeof(struct ib_cq_init_attr));
	cq_attr.cqe = 4;
	scq = ib_create_cq(device, NULL, NULL, NULL, &cq_attr);
	rcq = ib_create_cq(device, NULL, NULL, NULL, &cq_attr);
	if (!scq || !rcq) {
		pr_err("Fail to create CQ\n");
		goto free_pd;
	}

	/* QP */
	memset(&qp_init_attr, 0, sizeof(struct ib_qp_init_attr));
	qp_init_attr.send_cq = scq;
	qp_init_attr.recv_cq = rcq;
	qp_init_attr.cap.max_send_wr = 4;
	qp_init_attr.cap.max_recv_wr = 4;
	qp_init_attr.cap.max_send_sge = 1;
	qp_init_attr.cap.max_recv_sge = 1;
	qp_init_attr.sq_sig_type = IB_SIGNAL_ALL_WR;
	qp_init_attr.qp_type = qp_type;
	//qp_init_attr.create_flags = IB_QP_CREATE_USE_GFP_NOIO;
	comm->qp = ib_create_qp(comm->pd, &qp_init_attr);
	if (!comm->qp || IS_ERR(comm->qp)) {
		pr_err("Fail to create QP\n");
		goto free_cq;
	}
	pr_info("QPN=%d\n", comm->qp->qp_num);
	/* Just shortcut for local loop test */
	comm->dqpn = comm->qp->qp_num;

	/* MR */
	comm->mr = ib_get_dma_mr(comm->pd, IB_ACCESS_LOCAL_WRITE);
	if (IS_ERR(comm->mr)) {
		pr_err("Fail to create MR\n");
		goto free_qp;
	}

	if (device_create_file(device->dma_device, &dev_attr_sport) ||
	    device_create_file(device->dma_device, &dev_attr_buf_sz) ||
	    device_create_file(device->dma_device, &dev_attr_dlid) ||
	    device_create_file(device->dma_device, &dev_attr_dqpn) ||
	    device_create_file(device->dma_device, &dev_attr_send) ||
	    device_create_file(device->dma_device, &dev_attr_recv)) {
		pr_err("Fail to create sysfs\n");
		goto free_mr;
	}

	comm->dev = device;
	return;

free_mr:
	ib_dereg_mr(comm->mr);

free_qp:
	ib_destroy_qp(comm->qp);

free_cq:
	if (rcq)
		ib_destroy_cq(rcq);
	if (scq)
		ib_destroy_cq(scq);

free_pd:
	ib_dealloc_pd(comm->pd);
	comm_idx--;
}

static void clean_hw_resources(struct comm *comm)
{
	/* Check if already cleaned */
	if (comm->dev == NULL)
		return;

	if (comm->mr)
		ib_dereg_mr(comm->mr);
	if (comm->qp)
		ib_destroy_qp(comm->qp);
	if (comm->qp->send_cq)
		ib_destroy_cq(comm->qp->send_cq);
	if (comm->qp->recv_cq)
		ib_destroy_cq(comm->qp->recv_cq);
	if (comm->pd)
		ib_dealloc_pd(comm->pd);
}

static void clean_sysfs(struct comm *comm)
{
	/* Check if already cleaned */
	if (comm->dev == NULL)
		return;

	if (comm->dev->dma_device == NULL)
		return;

	device_remove_file(comm->dev->dma_device, &dev_attr_sport);
	device_remove_file(comm->dev->dma_device, &dev_attr_buf_sz);
	device_remove_file(comm->dev->dma_device, &dev_attr_dlid);
	device_remove_file(comm->dev->dma_device, &dev_attr_dqpn);
	device_remove_file(comm->dev->dma_device, &dev_attr_send);
	device_remove_file(comm->dev->dma_device, &dev_attr_recv);
}

static void clean_comm_entry(struct comm *comm)
{
	clean_hw_resources(comm);
	clean_sysfs(comm);

	comm->dev = NULL;
}

static void remove_one(struct ib_device *device, void *client_data)
{
	struct comm *comm = find_comm_obj(device->dma_device);

	pr_info("Pingpong.remove_one: %s\n", device->name);

	if (comm)
		clean_comm_entry(comm);
}

static void clean_comm_array(void)
{
	while (comm_idx >=0) {
		clean_comm_entry(&comm_array[comm_idx]);
		comm_idx--;
	}
}

static void init_comm_array(void)
{
	int i;

	for (i = 0; i < MAX_DEVS; i++)
		memset(&comm_array[i], 0, sizeof(struct comm));
}

static struct ib_client client = {
	.name   = "pingpong",
	.add    = add_one,
	.remove = remove_one
};

static int init_pingpong(void)
{
	int rc;

	init_comm_array();

	rc = ib_register_client(&client);
	if (rc) {
		pr_err("Fail to register IB client\n");
		return -ENODEV;
	}

	pr_info("Pingpong driver loaded successfully\n");

	return 0;
}

static void exit_pingpong(void)
{
	ib_unregister_client(&client);

	clean_comm_array();

	pr_info("Pingpong driver unloaded\n");
}

module_init(init_pingpong);
module_exit(exit_pingpong);
