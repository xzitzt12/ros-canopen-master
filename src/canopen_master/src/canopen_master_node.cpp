/**
 * @file canopen_master_node.cpp
 * @author zhangteng (xzitzt@163.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ros/ros.h"
#include "canopen_master/write_sdo.h"
#include "canopen_master/read_sdo.h"

#include "canfestival/canfestival.h"
#include "canopen_master.h"
#include <unistd.h>

static char canfestival_lib_path[] = "/usr/lib/libcanfestival_can_canalyst_ii.so";
static CO_Data *canopen_master_od_data;
static s_BOARD board = {(char *)"0", (char *)"500K"};
volatile int sdo_write_finished = 0;
volatile int sdo_read_finished = 0;
UNS32 sdo_read_data = 0;

void canopen_master_initialisation(CO_Data *d);
void canopen_master_preOperational(CO_Data *d);
void canopen_master_operational(CO_Data *d);
void canopen_master_stopped(CO_Data *d);
void canopen_master_post_sync(CO_Data *d);
void canopen_master_post_TPDO(CO_Data *d);
void canopen_master_post_SlaveBootup(CO_Data *d, UNS8 nodeid);
void canopen_master_init(CO_Data *d, UNS32 id);
void canopen_master_write(int nodeid, int index, int subindex, int size, int data);
bool canopen_master_write_sdo_server(canopen_master::write_sdo::Request &req,
                                canopen_master::write_sdo::Response &res);
bool canopen_master_read_sdo_server(canopen_master::read_sdo::Request &req,
                                canopen_master::read_sdo::Response &res);

int main(int argc, char **argv)
{
    canopen_master_od_data = &canopen_master_Data;
    sdo_write_finished = 0;
    sdo_read_finished = 0;

    /* 初始化CanFestival */
    TimerInit();

    if (LoadCanDriver(canfestival_lib_path) == NULL) {
        printf("Can not load canfestival library  %s\r\n", canfestival_lib_path);
        return 1;
    }

    canopen_master_od_data->initialisation = canopen_master_initialisation;
	canopen_master_od_data->preOperational = canopen_master_preOperational;
	canopen_master_od_data->operational = canopen_master_operational;
	canopen_master_od_data->stopped = canopen_master_stopped;
	canopen_master_od_data->post_sync = canopen_master_post_sync;
	canopen_master_od_data->post_TPDO = canopen_master_post_TPDO;
	canopen_master_od_data->post_SlaveBootup=canopen_master_post_SlaveBootup;

    if (!canOpen(&board, canopen_master_od_data)) {
        printf("Canopen init failed\r\n");
        return -1;
    }

    setNodeId(canopen_master_od_data, 1);
    StartTimerLoop(&canopen_master_init);
    sleep(5);

    printf("start\r\n");
    masterSendNMTstateChange(canopen_master_od_data, 2, NMT_Reset_Node);
    // sleep(1);
    masterSendNMTstateChange(canopen_master_od_data, 2, NMT_Start_Node);
    // sleep(1);
    canopen_master_write(2, 0x6040, 0x00, 2, 0x01);
    // sleep(1);
    canopen_master_write(2, 0x6040, 0x00, 2, 0x03);
    // sleep(1);
    canopen_master_write(2, 0x6040, 0x00, 2, 0x0F);
    // sleep(1);

    canopen_master_write(2, 0x6060, 0x00, 1, 0x01); // 位置模式
    // sleep(1);
    canopen_master_write(2, 0x6081, 0x00, 4, 0x64); // 目标速度
    // sleep(1);
    canopen_master_write(2, 0x6083, 0x00, 2, 0x03E8); // 加速度
    // sleep(1);
    canopen_master_write(2, 0x6084, 0x00, 2, 0x03E8); // 减速度
    // sleep(1);

    canopen_master_write(2, 0x607A, 0x00, 4, 0); // 目标位置
    //canopen_master_write(2, 0x607A, 0x00, 4, 0x1927c0); // 目标位置
    // sleep(1);
    canopen_master_write(2, 0x6040, 0x00, 2, 0x1F); // 启动
    // sleep(1);
    canopen_master_write(2, 0x6040, 0x00, 2, 0x0F); // 清除
    // sleep(1);

    // ros 节点初始化
    ros::init(argc, argv, "canopen_master");
    ros::NodeHandle n;
    ros::ServiceServer write_sdo_service = n.advertiseService("write_sdo", canopen_master_write_sdo_server);
    ros::ServiceServer read_sdo_service = n.advertiseService("read_sdo", canopen_master_read_sdo_server);

    ros::spin();

    for(;;) {
        usleep(1000000);
    }

    return 0;
}

void canopen_master_initialisation(CO_Data *d)
{
    printf("Node initialisation\r\n");
}

void canopen_master_preOperational(CO_Data *d)
{
    printf("Node preOperational\r\n");
}

void canopen_master_operational(CO_Data *d)
{
    printf("Node operational\r\n");
}

void canopen_master_stopped(CO_Data *d)
{
    printf("Node stopped\r\n");
}

void canopen_master_post_sync(CO_Data *d)
{
    // printf("Node post sync\r\n");
}

void canopen_master_post_TPDO(CO_Data *d)
{
    // printf("Node pot tpdo\r\n");
}

void canopen_master_post_SlaveBootup(CO_Data *d, UNS8 nodeid)
{
    printf("Slave 0x%x boot up\r\n", nodeid);
}

void canopen_master_init(CO_Data *d, UNS32 id)
{
    if (board.baudrate) {
        setState(canopen_master_od_data, Initialisation);
    }

    setState(canopen_master_od_data, Pre_operational);
    while (getState(canopen_master_od_data) != Pre_operational) {
        ;
    }
}

void canopen_master_check_write_sdo(CO_Data *d, UNS8 nodeid)
{
    UNS32 abort_code;

    if (getWriteResultNetworkDict(canopen_master_od_data, nodeid, &abort_code) != SDO_FINISHED) {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abort_code);
    } else {
        printf("\nSend data OK\n");
    }

    /* Finalize last SDO transfer with this node */
    closeSDOtransfer(canopen_master_od_data, nodeid, SDO_CLIENT);

    sdo_write_finished = 1; // 写操作完成
}

void canopen_master_write(int nodeid, int index, int subindex, int size, int data)
{
    while (sdo_write_finished != 0) {
        usleep(10000);
    }
    sdo_write_finished = 0;
    
    writeNetworkDictCallBack(canopen_master_od_data, nodeid, index, subindex, size, 0, &data, canopen_master_check_write_sdo, 0);

    // 等待写完成
    while (sdo_write_finished == 0) {
        usleep(10000);
    }
    sdo_write_finished = 0;
}

void canopen_master_check_read_sdo(CO_Data *d, UNS8 nodeid)
{
    UNS32 abort_code;
    UNS32 size = 64;

    if (getReadResultNetworkDict(canopen_master_od_data, nodeid, &sdo_read_data, &size, &abort_code) != SDO_FINISHED) {
        printf("\nResult : Failed in getting information for slave %2.2x, AbortCode :%4.4x \n", nodeid, abort_code);
    }

    closeSDOtransfer(canopen_master_od_data, nodeid, SDO_CLIENT);

    sdo_read_finished = 1;
}

void canopen_master_read(int nodeid, int index, int subindex, int size, int32_t *data)
{
    while (sdo_read_finished != 0) {
        usleep(10000);
    }
    sdo_read_finished = 0;

    readNetworkDictCallback(canopen_master_od_data, nodeid, index, subindex, 0, canopen_master_check_read_sdo, 0);

    // 等待读完成
    while (sdo_read_finished == 0) {
        usleep(10000);
    }

    *data = sdo_read_data;

    sdo_read_finished = 0;
}

bool canopen_master_write_sdo_server(canopen_master::write_sdo::Request &req,
                                canopen_master::write_sdo::Response &res)
{
    canopen_master_write(req.nodeid,req.index,req.subindex,req.size,req.data);
    res.error = 0;

    return true;
}

bool canopen_master_read_sdo_server(canopen_master::read_sdo::Request &req,
                                canopen_master::read_sdo::Response &res)
{
    int32_t data;

    canopen_master_read(req.nodeid, req.index, req.subindex, req.size, &data);
    res.error = 0;
    res.data = data;

    return true;
}




