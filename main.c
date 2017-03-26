#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <rdma/ib_verbs.h>

#define DRIVER_AUTHOR "Yuval Shaia <yuval.shaia@oracle.com>"
#define DRIVER_DESC   "Pingpong test"
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

#define MAX_BUFS 10

struct ib_device *dev;
struct ib_pd *pd;
struct ib_qp *qp;
bool qp_initialized;
union ib_gid dgid;
struct ib_mr *mr;
char *send_buf;
u64 send_buf_dma_addr;
char *recv_buf[MAX_BUFS];
u64 recv_buf_dma_addr[MAX_BUFS];
struct ib_ah *vah;

struct comp_work {
	struct work_struct work;
};

int qp_type = IB_QPT_RC;
module_param_named(qp_type, qp_type, int, 0444);
MODULE_PARM_DESC(qp_type, "QP Type");
int sport = 1;
module_param_named(sport, sport, int, 0444);
MODULE_PARM_DESC(sport, "Index of port to use for test");
unsigned long cycles = 0;
module_param_named(cycles, cycles, ulong, 0444);
MODULE_PARM_DESC(cycles, "Number of cycles to run");
int sender = 1;
module_param_named(sender, sender, int, 0444);
MODULE_PARM_DESC(sender, "Sender");
int receiver = 1;
module_param_named(receiver, receiver, int, 0444);
MODULE_PARM_DESC(receiver, "Receiver");
ulong dguid = 0;
module_param_named(dguid, dguid, ulong, 0444);
MODULE_PARM_DESC(dguid, "Target GUID");
int dqpn = -1;
module_param_named(dqpn, dqpn, int, 0444);
MODULE_PARM_DESC(dqpn, "Target QPN");
int buf_sz = 4096;
module_param_named(buf_sz, buf_sz, int, 0444);
MODULE_PARM_DESC(buf_sz, "Buffer size");

unsigned long sends_counter = 1;

static struct workqueue_struct *wq;
static void tx_comp_processor(struct work_struct *work);
static void rx_comp_processor(struct work_struct *work);

static int init_qp_state(struct ib_qp *qp, bool rts, int port,
			 union ib_gid dgid, u32 dqpn)
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
	if (qp_type == IB_QPT_RC) {
		qp_attr_mask |= IB_QP_ACCESS_FLAGS;
		qp_attr.qp_access_flags = IB_ACCESS_REMOTE_WRITE |
  					  IB_ACCESS_REMOTE_READ;
	}
	if (qp_type == IB_QPT_UD) {
		qp_attr_mask |= IB_QP_QKEY;
		qp_attr.qkey = 0;
	}
	rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
	if (rc)
		goto err;

	/* RTR */
	qp_attr_mask = 0;
	qp_attr_mask |= IB_QP_STATE;
	qp_attr.qp_state = IB_QPS_RTR;
	if (qp_type == IB_QPT_RC) {
		qp_attr_mask |= IB_QP_AV;
		qp_attr.ah_attr.port_num = port;
		qp_attr.ah_attr.sl = 0;
		qp_attr.ah_attr.ah_flags = 0;
		qp_attr.ah_attr.grh.dgid = dgid;
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
	}
	rc = ib_modify_qp(qp, &qp_attr, qp_attr_mask);
	if (rc)
		goto err;

	/* RTS */
	if (rts) {
		qp_attr_mask = 0;
		qp_attr_mask |= IB_QP_STATE;
		qp_attr_mask |= IB_QP_SQ_PSN;
		qp_attr.sq_psn = psn;
		qp_attr.qp_state = IB_QPS_RTS;
		if (qp_type == IB_QPT_RC) {
			qp_attr_mask |= IB_QP_TIMEOUT;
			qp_attr.timeout = 0x4;
			qp_attr_mask |= IB_QP_RETRY_CNT;
			qp_attr.retry_cnt = 0;
			qp_attr_mask |= IB_QP_RNR_RETRY;
			qp_attr.rnr_retry = 0;
			qp_attr_mask |= IB_QP_MAX_QP_RD_ATOMIC;
			qp_attr.max_rd_atomic = 1;
		}
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

static void post_send(void)
{
	struct ib_send_wr wr, *bad_wr;
	struct ib_sge sge;
	int rc;

	if (++sends_counter > cycles)
		return;

	pr_info("Sending %ld/%ld\n", sends_counter, cycles);

	sprintf(send_buf, "%ld", sends_counter);

	sge.addr = send_buf_dma_addr;
	sge.length = buf_sz;
	sge.lkey = mr->lkey;

	rc = ib_req_notify_cq(qp->send_cq, IB_CQ_NEXT_COMP);
	if (rc < 0)
		pr_err("ib_req_notify_cq returned %d\n", rc);

	memset(&wr, 0, sizeof(wr));
	wr.next = NULL;
	wr.wr_id = (unsigned long)send_buf;
	pr_info("send.wr_id=%lld\n", wr.wr_id);
	wr.opcode = IB_WR_SEND;
	wr.send_flags = IB_SEND_SIGNALED;
	wr.sg_list = &sge;
	wr.num_sge = 1;

	if (qp_type == IB_QPT_RC)
		rc = ib_post_send(qp, &wr, &bad_wr);
	if (qp_type == IB_QPT_UD) {
		struct ib_ud_wr ud_wr;
		memcpy(&ud_wr.wr, &wr, sizeof(wr));
		ud_wr.remote_qpn = dqpn;
		pr_err("remote_qpn=%d\n", ud_wr.remote_qpn);
		ud_wr.ah = vah;
		rc = ib_post_send(qp, &ud_wr.wr, &bad_wr);
	}
	if (rc)
		pr_err("ib_post_send returned %d\n", rc);

	return;
}

static void post_recv(size_t buf_idx)
{
	struct ib_recv_wr wr, *bad_wr;
	struct ib_sge sge;
	int rc;

	sge.addr = recv_buf_dma_addr[buf_idx];
	sge.length = buf_sz;
	sge.lkey = mr->lkey;

	wr.next = NULL;
	wr.wr_id = buf_idx;
	pr_info("recv.wr_id=%lld\n", wr.wr_id);
	wr.sg_list = &sge;
	wr.num_sge = 1;

	rc = ib_req_notify_cq(qp->recv_cq, IB_CQ_NEXT_COMP);
	if (rc < 0)
		pr_err("ib_req_notify_cq returned %d\n", rc);

	rc = ib_post_recv(qp, &wr, &bad_wr);
	if (rc)
		pr_err("ib_post_recv returned %d\n", rc);
}

static int init_communication_attrs(void)
{
	int rc;
	struct ib_gid_attr attr = {0};

	if (!dgid.global.interface_id) {
		rc = ib_query_gid(dev, sport, 0, &dgid, &attr);
	} else {
		/* Just to fetch subnet prefix */
		union ib_gid dgid;
		rc = ib_query_gid(dev, sport, 0, &dgid, &attr);
		dgid.global.subnet_prefix = dgid.global.subnet_prefix;
	}
	if (rc) {
		pr_err("Fail to query gid %d (err=%d)\n", sport, rc);
		return rc;
	}

	pr_info("sport=%d\n", sport);
	pr_info("dgid=0x%llx,0x%llx\n", be64_to_cpu(dgid.global.interface_id),
		be64_to_cpu(dgid.global.subnet_prefix));

	return 0;
}

static void init_and_send(struct device *d)
{
	int rc;

	rc = init_communication_attrs();
	if (rc) {
		pr_err("Fail to set communication parameters\n");
		return;
	}

	if (!qp_initialized) {
		rc = init_qp_state(qp, true, sport, dgid, dqpn);
		if (rc) {
			pr_err("Fail to change QP state to send\n");
			return;
		}
		qp_initialized = true;
	}

	sends_counter = 0;

	post_send();
}

static void init_and_recv(struct device *d)
{
	int rc, i;

	rc = init_communication_attrs();
	if (rc) {
		pr_err("Fail to set communication parameters\n");
		return;
	}

	if (!qp_initialized) {
		rc = init_qp_state(qp, true, sport, dgid, dqpn);
		if (rc) {
			pr_err("Fail to change QP state to recv\n");
			return;
		}
		qp_initialized = true;
	}

	for (i = 0; i < MAX_BUFS; i++)
		post_recv(i);
}

static void rx_comp_processor(struct work_struct *work)
{
	struct ib_wc wc;

	pr_info("RX completion\n");

	ib_poll_cq(qp->recv_cq, 1, &wc);
	pr_info("wc.status=%d\n", wc.status);
	pr_info("wc.wr_id=%lld\n", wc.wr_id);

	if (wc.status != IB_WC_SUCCESS) {
		pr_err("wc.vendor_err=0x%x\n", wc.vendor_err);
		/*
		if ((wc.status == IB_WC_REM_OP_ERR) &&
		    (wc.vendor_err == 0x102)) {
			pr_err("post_recv done\n");
			mdelay(1000);
		} else {
			goto out;
		}
		*/
		goto out;
	}

	pr_info("payload=%s\n", recv_buf[wc.wr_id]);
	post_recv(wc.wr_id);

out:
	kfree(work);
}

void rx_comp_handler(struct ib_cq *cq, void *cq_context)
{
	struct work_struct *work = kmalloc(sizeof *work, GFP_ATOMIC);

	INIT_WORK(work, rx_comp_processor);

	queue_work(wq, work);
}

static void tx_comp_processor(struct work_struct *work)
{
	struct ib_wc wc;

	pr_info("TX completion\n");

	ib_poll_cq(qp->send_cq, 1, &wc);
	pr_info("wc.status=%d\n", wc.status);
	pr_info("wc.wr_id=%lld\n", wc.wr_id);

	if (wc.status != IB_WC_SUCCESS) {
		pr_err("wc.vendor_err=0x%x\n", wc.vendor_err);
		/*
		if ((wc.status == IB_WC_REM_OP_ERR) &&
		    (wc.vendor_err == 0x102)) {
			pr_err("post_recv done\n");
			mdelay(1000);
		} else {
			goto out;
		}
		*/
		goto out;
	}

	post_send();

out:
	kfree(work);
}

void tx_comp_handler(struct ib_cq *cq, void *cq_context)
{
	struct work_struct *work = kmalloc(sizeof *work, GFP_ATOMIC);

	INIT_WORK(work, tx_comp_processor);

	queue_work(wq, work);
}

static void free_buffers(void)
{
	int i;

	if (send_buf) {
		ib_dma_unmap_single(dev, send_buf_dma_addr, buf_sz,
				    DMA_TO_DEVICE);
		kfree(send_buf);
		send_buf = NULL;
	}

	for (i = 0; i < MAX_BUFS; i++) {
		if (recv_buf[i]) {
			ib_dma_unmap_single(dev, recv_buf_dma_addr[i], buf_sz,
					    DMA_TO_DEVICE);
			kfree(recv_buf[i]);
			recv_buf[i] = NULL;
		}
	}
}

static int alloc_buffers(void)
{
	int i;

	if (sender) {
		send_buf = kmalloc(buf_sz, GFP_KERNEL);
		if (!send_buf)
			goto out_err;

		send_buf_dma_addr = ib_dma_map_single(dev, send_buf, buf_sz,
						      DMA_FROM_DEVICE);
		if (ib_dma_mapping_error(dev, send_buf_dma_addr))
			goto out_err;
	}

	if (receiver) {
		for (i = 0; i < MAX_BUFS; i++) {
			recv_buf[i] = kmalloc(buf_sz, GFP_KERNEL);
			if (!recv_buf[i])
				goto out_err;

			recv_buf_dma_addr[i] =
				ib_dma_map_single(dev, recv_buf[i], buf_sz,
						  DMA_FROM_DEVICE);
			if (ib_dma_mapping_error(dev, recv_buf_dma_addr[i]))
				goto out_err;

			sprintf(recv_buf[i], "INIT_DATA_%d", i);
		}
	}

	return 0;

out_err:
	free_buffers();
	return -ENOMEM;
}

static void add_one(struct ib_device *device)
{
	struct ib_cq *scq = 0, *rcq = 0;
	struct ib_qp_init_attr qp_init_attr = {0};
	struct ib_cq_init_attr cq_attr = {0};
	struct ib_ah_attr av_attr;

	pr_info("%s\n", device->name);

	if (dev)
		return;

	pr_info("Pingpong.add_one: %s\n", device->name);

	dev = device;
	dgid.global.subnet_prefix = 0x80fe;
	dgid.global.interface_id = cpu_to_be64(dguid);

	if (init_communication_attrs()) {
		pr_err("Fail to set communication parameters\n");
		return;
	}

	if (alloc_buffers()) {
		pr_err("Fail to alloc recv and send buffers\n");
		return;
	}

	/* PD */
	pd = ib_alloc_pd(device);
	if (!pd)
		goto free_bufs;

	/* AH */
	if (qp_type == IB_QPT_UD) {
		av_attr.ah_flags |= IB_AH_GRH;
		av_attr.grh.dgid = dgid;
		av_attr.port_num = sport;
		vah = ib_create_ah(pd, &av_attr);
		if (IS_ERR(vah)) {
			pr_err("Fail to create AH\n");
			goto free_pd;
		}
	}

	/* CQs */
	cq_attr.cqe = 4;
	scq = ib_create_cq(device, tx_comp_handler, NULL, NULL, &cq_attr);
	rcq = ib_create_cq(device, rx_comp_handler, NULL, NULL, &cq_attr);
	if (!scq || !rcq) {
		pr_err("Fail to create CQ\n");
		goto free_ah;
	}

	/* QP */
	qp_init_attr.send_cq = scq;
	qp_init_attr.recv_cq = rcq;
	qp_init_attr.cap.max_send_wr = 4;
	qp_init_attr.cap.max_recv_wr = 4;
	qp_init_attr.cap.max_send_sge = 1;
	qp_init_attr.cap.max_recv_sge = 1;
	qp_init_attr.sq_sig_type = IB_SIGNAL_ALL_WR;
	qp_init_attr.qp_type = qp_type;
	//qp_init_attr.create_flags = IB_QP_CREATE_USE_GFP_NOIO;
	qp = ib_create_qp(pd, &qp_init_attr);
	if (!qp || IS_ERR(qp)) {
		pr_err("Fail to create QP\n");
		goto free_cq;
	}

	if (dqpn == -1) {
		pr_info("QPN=%d\n", qp->qp_num);
		/* Just shortcut for local loop test */
		dqpn = qp->qp_num;
	}

	/* MR */
	mr = ib_get_dma_mr(pd, IB_ACCESS_LOCAL_WRITE);
	if (IS_ERR(mr)) {
		pr_err("Fail to create MR\n");
		goto free_qp;
	}

	if (receiver)
		init_and_recv(device->dma_device);
	if (sender)
		init_and_send(device->dma_device);

	return;

free_qp:
	ib_destroy_qp(qp);

free_cq:
	if (rcq)
		ib_destroy_cq(rcq);
	if (scq)
		ib_destroy_cq(scq);

free_ah:
	if (qp_type == IB_QPT_UD)
		ib_destroy_ah(vah);

free_pd:
	ib_dealloc_pd(pd);

free_bufs:
	free_buffers();
}

static void clean_hw_resources(void)
{
	/* Check if already cleaned */
	if (!dev)
		return;

	if (mr)
		ib_dereg_mr(mr);
	if (qp)
		ib_destroy_qp(qp);
	if (qp->send_cq)
		ib_destroy_cq(qp->send_cq);
	if (qp->recv_cq)
		ib_destroy_cq(qp->recv_cq);
	if (pd)
		ib_dealloc_pd(pd);

	dev = NULL;
}

static void remove_one(struct ib_device *device, void *client_data)
{
	clean_hw_resources();
	pr_info("Pingpong.remove_one: %s\n", device->name);
}

static struct ib_client client = {
	.name   = "pingpong",
	.add    = add_one,
	.remove = remove_one
};

static int init_pingpong(void)
{
	int rc;

	dev = NULL;

	wq = alloc_workqueue("pingpong_comp", 0, 0);
	if (!wq) {
		pr_err("Fail to create completion WQ\n");
		return -ENODEV;
	}

	rc = ib_register_client(&client);
	if (rc)
		goto err_fail_reg_ib_client;

	pr_info("Pingpong driver loaded successfully\n");

	return 0;

err_fail_reg_ib_client:
	pr_err("Fail to register IB client\n");
	destroy_workqueue(wq);

	return -ENODEV;
}

static void exit_pingpong(void)
{
	ib_unregister_client(&client);

	destroy_workqueue(wq);

	pr_info("Pingpong driver unloaded\n");
}

module_init(init_pingpong);
module_exit(exit_pingpong);
