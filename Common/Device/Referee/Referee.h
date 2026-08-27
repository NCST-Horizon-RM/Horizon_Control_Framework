#ifndef HORIZON_REFEREE_H
#define HORIZON_REFEREE_H

#include "usart.h"
#include "string.h"
#include "stdint.h"
#include "CRC_DJI.h"
#include "Offline_Detector.h"

/* 帧长度 */
#define REFEREE_RXFRAME_LENGTH 256
#define REFEREE_MAX_PACKET_SIZE 136 // 裁判系统单包最大长度
#define FrameHeader_Length 5U
#define CMDID_Length 2U
#define CRC16_Length 2U

/* ==================== CMD ID ==================== */
enum Read_Cmd_ID_Typdef
{
    game_state                 = 0x0001, /* 0x0001: 比赛状态数据，固定以1Hz频率发送 */
    Match_results              = 0x0002, /* 0x0002: 比赛结果数据，比赛结束触发发送 */
    Robot_HP                   = 0x0003, /* 0x0003: 机器人血量数据，固定以3Hz频率发送 */
    Venue_Events               = 0x0101, /* 0x0101: 场地事件数据，固定以1Hz频率发送 */
    Referee_warning            = 0x0104, /* 0x0104: 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送 */
    Dart_fire                  = 0x0105, /* 0x0105: 飞镖发射相关数据，固定以1Hz频率发送 */
    Robot_performan            = 0x0201, /* 0x0201: 机器人性能体系数据，固定以10Hz频率发送 */
    time_power                 = 0x0202, /* 0x0202: 实时底盘缓冲能量和射击热量数据，固定以10Hz频率发送 */
    Robot_location             = 0x0203, /* 0x0203: 机器人位置数据，固定以1Hz频率发送 */
    Robot_buff                 = 0x0204, /* 0x0204: 机器人增益和底盘能量数据，固定以3Hz频率发送 */
    Damage_status              = 0x0206, /* 0x0206: 伤害状态数据，伤害发生后发送 */
    time_shooting              = 0x0207, /* 0x0207: 实时射击数据，弹丸发射后发送 */
    Allowable_ammo             = 0x0208, /* 0x0208: 允许发弹量，固定以10Hz频率发送 */
    RFID_status                = 0x0209, /* 0x0209: 机器人 RFID 模块状态，固定以3Hz频率发送 */
    Dart_directives            = 0x020A, /* 0x020A: 飞镖选手端指令数据，固定以3Hz频率发送 */
    Ground_location            = 0x020B, /* 0x020B: 地面机器人位置数据，固定以1Hz频率发送 */
    Radar_Marking              = 0x020C, /* 0x020C: 雷达标记进度数据，固定以1Hz频率发送 */
    Route_Informat             = 0x020D, /* 0x020D: 哨兵自主决策信息同步，固定以1Hz频率发送 */
    Radar_Informat             = 0x020E, /* 0x020E: 雷达自主决策信息同步，固定以1Hz频率发送 */

    Robot_Interaction          = 0x0301, /* 0x0301: 机器人交互数据，发送方触发发送，频率上限为30Hz */
    Custom_controller_to_robot = 0x0302, /* 0x0302: 自定义控制器与机器人交互数据，发送方触发发送，频率上限为30Hz */
    Minimap                    = 0x0303, /* 0x0303: 选手端小地图交互数据，选手端触发发送 */
    Map_receive_radar          = 0x0305, /* 0x0305: 选手端小地图接收雷达数据，频率上限为5Hz */
    Custom_controller_to_map   = 0x0306, /* 0x0306: 自定义控制器与选手端交互数据，发送方触发发送，频率上限为30Hz */
    Map_receive_path           = 0x0307, /* 0x0307: 选手端小地图接收路径数据，频率上限为1Hz */
    Map_receive_robot_info     = 0x0308, /* 0x0308: 选手端小地图接收机器人数据，频率上限为3Hz */
    Robot_to_custom_controller = 0x0309, /* 0x0309: 自定义控制器接收机器人数据，频率上限为10Hz */
    Robot_to_custom_client     = 0x0310, /* 0x0310: 机器人发送给自定义客户端的数据，频率上限为50Hz */
    Custom_client_to_robot     = 0x0311  /* 0x0311: 自定义客户端发送给机器人的自定义指令，频率上限为75Hz */
};

typedef struct __packed
{
    uint8_t  SOF;         /* 数据帧起始字节,固定值为0xA5 */
    uint16_t DataLenth;   /* 数据帧中 data 的长度 */
    uint8_t  Seq;         /* 包序号 */
    uint8_t  CRC8;        /* 帧头 CRC8 校验 */
} frame_header_R_Typdef;

typedef struct __packed
{
    uint8_t game_type : 4;         /* bit 0-3:比赛类型。1: RoboMaster 机甲大师超级对抗赛, 2: RoboMaster 机甲大师高校单项赛, 3: ICRA RoboMaster 高校人工智能挑战赛, 4: RoboMaster 机甲大师高校联盟赛3V3对抗, 5: RoboMaster 机甲大师高校联盟赛步兵对抗 */
    uint8_t game_progress : 4;     /* bit 4-7:当前比赛阶段。0:未开始比赛, 1:准备阶段, 2:十五秒裁判系统自检阶段, 3: 五秒倒计时, 4:比赛中, 5:比赛结算中 */
    uint16_t stage_remain_time;    /* 当前阶段剩余时间,单位:秒 */
    uint64_t SyncTimeStamp;        /* UNIX时间,当机器人正确连接到裁判系统的NTP服务器后生效 */
} game_status_t;

typedef struct __packed
{
    uint8_t winner;                /* 0:平局, 1:红方胜利, 2:蓝方胜利 */
} game_result_t;

typedef struct __packed
{
    uint16_t ally_1_robot_HP;      /* 己方1号英雄机器人血量,若该机器人未上场或者被罚下,则血量为0,下文同理 */
    uint16_t ally_2_robot_HP;      /* 己方2号工程机器人血量 */
    uint16_t ally_3_robot_HP;      /* 己方3号步兵机器人血量 */
    uint16_t ally_4_robot_HP;      /* 己方4号步兵机器人血量 */
    uint16_t reserved;             /* 保留位 */
    uint16_t ally_7_robot_HP;      /* 己方7号哨兵机器人血量 */
    uint16_t ally_outpost_HP;      /* 己方前哨站血量 */
    uint16_t ally_base_HP;         /* 己方基地血量 */
} game_robot_HP_t;

/* 0x0101 场地事件数据 */
typedef struct __packed
{
    uint32_t supply_zone_status : 1;                 /* bit 0:己方补给区的占领状态,1为已占领 */
    uint32_t reserved_1 : 1;                         /* bit 1:保留位 */
    uint32_t supply_zone_status_rmul : 1;            /* bit 2:己方补给区的占领状态,1为已占领(仅RMUL适用) */
    uint32_t small_energy_mechanism_status : 2;      /* bit 3-4:己方小能量机关的激活状态,0为未激活,1为已激活,2为正在激活 */
    uint32_t big_energy_mechanism_status : 2;        /* bit 5-6:己方大能量机关的激活状态,0为未激活,1为已激活,2为正在激活 */
    uint32_t central_highland_status : 2;            /* bit 7-8:己方中央高地的占领状态,1为被己方占领,2为被对方占领 */
    uint32_t trapezoidal_highland_status : 2;        /* bit 9-10:己方梯形高地的占领状态,1为已占领 */
    uint32_t dart_hit_time : 9;                      /* bit 11-19:对方飞镖最后一次击中己方前哨站或基地的时间(0-420, 开局默认为0) */
    uint32_t dart_hit_target : 3;                    /* bit 20-22:对方飞镖最后一次击中己方前哨站或基地的具体目标,开局默认为0,1为击中前哨站,2为击中基地固定目标,3为击中基地随机固定目标,4为击中基地随机移动目标,5为击中基地末端移动目标 */
    uint32_t center_buff_status : 2;                 /* bit 23-24:中心增益点的占领状态,0为未被占领,1为被己方占领,2为被对方占领,3为被双方占领。(仅RMUL适用) */
    uint32_t fortress_buff_status : 2;               /* bit 25-26:己方堡垒增益点的占领状态,0为未被占领,1为被己方占领,2为被对方占领,3为被双方占领 */
    uint32_t outpost_buff_status : 2;                /* bit 27-28:己方前哨站增益点的占领状态,0为未被占领,1为被己方占领,2为被对方占领 */
    uint32_t base_buff_status : 1;                   /* bit 29:己方基地增益点的占领状态,1为已占领 */
    uint32_t reserved_2 : 2;                         /* bit 30-31:保留位 */
} event_data_t;

typedef struct __packed
{
    uint8_t level;                  /* 己方最后一次受到判罚的等级: 1:双方黄牌 2:黄牌 3:红牌 4:判负 */
    uint8_t offending_robot_id;     /* 己方最后一次受到判罚的违规机器人ID(判负和双方黄牌时,该值为0) */
    uint8_t count;                  /* 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数(开局默认为0) */
} referee_warning_t;

/* 0x0105 飞镖发射相关数据 */
typedef struct __packed
{
    uint8_t dart_remaining_time;        /* 己方飞镖发射剩余时间,单位:秒 */
    uint16_t dart_hit_target : 3;       /* bit 0-2:最近一次己方飞镖击中的目标,开局默认为0,1为击中前哨站,2为击中基地固定目标,3为击中基地随机固定目标,4为击中基地随机移动目标,5为击中基地末端移动目标 */
    uint16_t dart_hit_count : 3;        /* bit 3-5:对方最近被击中的目标累计被击中计次数,开局默认为0,至多为4 */
    uint16_t dart_selected_target : 3;  /* bit 6-8:飞镖此时选定的击打目标,开局默认或未选定/选定前哨站时为0,选中基地固定目标为1,选中基地随机固定目标为2,选中基地随机移动目标为3,选中基地末端移动目标为4 */
    uint16_t reserved : 7;              /* bit 9-15:保留 */
} dart_info_t;

/* 0x0201 机器人性能体系数据 */
typedef struct __packed
{
    uint8_t robot_id;                            /* 本机器人 ID */
    uint8_t robot_level;                         /* 机器人等级 */
    uint16_t current_HP;                         /* 机器人当前血量 */
    uint16_t maximum_HP;                         /* 机器人血量上限 */
    uint16_t shooter_barrel_cooling_value;       /* 机器人射击热量每秒冷却值 */
    uint16_t shooter_barrel_heat_limit;          /* 机器人射击热量上限 */
    uint16_t chassis_power_limit;                /* 机器人底盘功率上限 */
    uint8_t power_management_gimbal_output : 1;  /* bit 0: gimbal 输出,0为无输出,1为24V输出 */
    uint8_t power_management_chassis_output : 1; /* bit 1: chassis 口输出,0为无输出,1为24V输出 */
    uint8_t power_management_shooter_output : 1; /* bit 2: shooter 口输出,0为无输出,1为24V 输出 */
    uint8_t reserved_power_management : 5;
} robot_status_t;

typedef struct __packed
{
    uint16_t reserved_1;                         /* 保留位 */
    uint16_t reserved_2;                         /* 保留位 */
    float reserved_3;                            /* 保留位 */
    uint16_t buffer_energy;                      /* 缓冲能量(单位:J) */
    uint16_t shooter_17mm_barrel_heat;           /* 17mm 发射机构的射击热量 */
    uint16_t shooter_42mm_barrel_heat;           /* 42mm 发射机构的射击热量 */
} power_heat_data_t;

typedef struct __packed
{
    float x;                                     /* 本机器人位置x坐标,单位:m */
    float y;                                     /* 本机器人位置y坐标,单位:m */
    float angle;                                 /* 本机器人测速模块的朝向,单位:度,正北为0度 */
} robot_pos_t;

/* 0x0204 机器人增益和底盘能量数据 */
typedef struct __packed
{
    uint8_t recovery_buff;                       /* 机器人回血增益(百分比,值为10表示每秒恢复血量上限的10%) */
    uint16_t cooling_buff;                       /* 机器人射击热量冷却增益具体值(直接值,值为x表示热量冷却增加x/s) */
    uint8_t defence_buff;                        /* 机器人防御增益(百分比,值为50表示50%防御增益) */
    uint8_t vulnerability_buff;                  /* 机器人负防御增益(百分比,值为30表示-30%防御增益) */
    uint16_t attack_buff;                        /* 机器人攻击增益(百分比,值为50表示50%攻击增益) */
    uint8_t energy_125 : 1;                      /* bit 0:在剩余能量≥125%时为1,其余情况为0 */
    uint8_t energy_100 : 1;                      /* bit 1:在剩余能量≥100%时为1,其余情况为0 */
    uint8_t energy_50 : 1;                       /* bit 2:在剩余能量≥50%时为1,其余情况为0 */
    uint8_t energy_30 : 1;                       /* bit 3:在剩余能量≥30%时为1,其余情况为0 */
    uint8_t energy_15 : 1;                       /* bit 4:在剩余能量≥15%时为1,其余情况为0 */
    uint8_t energy_5 : 1;                        /* bit 5:在剩余能量≥5%时为1,其余情况为0 */
    uint8_t energy_1 : 1;                        /* bit 6:在剩余能量≥1%时为1,其余情况为0 */
    uint8_t energy_reserved : 1;
} buff_t;

typedef struct __packed
{
    uint8_t armor_id : 4;                        /* bit 0-3:当扣血原因为装甲模块被弾丸攻击、受撞击或离线时,该4 bit组成的数值为装甲模块或测速模块的ID编号;当其他原因导致扣血时,该数值为0 */
    uint8_t HP_deduction_reason : 4;             /* bit 4-7:血量变化类型。0:装甲模块被弹丸攻击导致扣血, 1:装甲模块或超级电容管理模块离线导致扣血, 5:装甲模块受到撞击导致扣血 */
} hurt_data_t;

/* 0x0207 实时射击数据 */
typedef struct __packed
{
    uint8_t reserved_1 : 1;
    uint8_t bullet_type_17mm : 1;                /* bit 1: 17mm 弹丸 */
    uint8_t bullet_type_42mm : 1;                /* bit 2: 42mm 弹丸 */
    uint8_t reserved_2 : 5;
    uint8_t shooter_number;                      /* 发射机构 ID: 1:17mm 发射机构, 2:保留位, 3:42mm 发射机构 */
    uint8_t launching_frequency;                 /* 弹丸射速(单位:Hz) */
    float initial_speed;                         /* 弹丸初速度(单位:m/s) */
} shoot_data_t;

typedef struct __packed
{
    uint16_t projectile_allowance_17mm;          /* 机器人自身拥有的17mm弹丸允许发弹量 */
    uint16_t projectile_allowance_42mm;          /* 42mm 弹丸允许发弹量 */
    uint16_t remaining_gold_coin;                /* 剩余金币数量 */
    uint16_t projectile_allowance_fortress;      /* 堡垒增益点提供的储备17mm弹丸允许发弹量;该值与机器人是否实际占领堡垒无关 */
} projectile_allowance_t;

/* 0x0209 机器人 RFID 模块状态 */
typedef struct __packed
{
    uint32_t ally_base : 1;                      /* bit 0:己方基地增益点 */
    uint32_t ally_central_highland : 1;          /* bit 1:己方中央高地增益点 */
    uint32_t enemy_central_highland : 1;         /* bit 2:对方中央高地增益点 */
    uint32_t ally_trapezoidal_highland : 1;      /* bit 3:己方梯形高地增益点 */
    uint32_t enemy_trapezoidal_highland : 1;     /* bit 4:对方梯形高地增益点 */
    uint32_t ally_fly_ramp_front : 1;            /* bit 5:己方地形跨越增益点(飞坡)(靠近己方一侧飞坡前) */
    uint32_t ally_fly_ramp_back : 1;             /* bit 6:己方地形跨越增益点(飞坡)(靠近己方一侧飞坡后) */
    uint32_t enemy_fly_ramp_front : 1;           /* bit 7:对方地形跨越增益点(飞坡)(靠近对方一侧飞坡前) */
    uint32_t enemy_fly_ramp_back : 1;            /* bit 8:对方地形跨越增益点(飞坡)(靠近对方一侧飞坡后) */
    uint32_t ally_central_highland_lower : 1;    /* bit 9:己方地形跨越增益点(中央高地下方) */
    uint32_t ally_central_highland_upper : 1;    /* bit 10:己方地形跨越增益点(中央高地上方) */
    uint32_t enemy_central_highland_lower : 1;   /* bit 11:对方地形跨越增益点(中央高地下方) */
    uint32_t enemy_central_highland_upper : 1;   /* bit 12:对方地形跨越增益点(中央高地上方) */
    uint32_t ally_highway_lower : 1;             /* bit 13:己方地形跨越增益点(公路下方) */
    uint32_t ally_highway_upper : 1;             /* bit 14:己方地形跨越增益点(公路上方) */
    uint32_t enemy_highway_lower : 1;            /* bit 15:对方地形跨越增益点(公路下方) */
    uint32_t enemy_highway_upper : 1;            /* bit 16:对方地形跨越增益点(公路上方) */
    uint32_t ally_fortress : 1;                  /* bit 17:己方堡垒增益点 */
    uint32_t ally_outpost : 1;                   /* bit 18:己方前哨站增益点 */
    uint32_t ally_supply_zone_not_overlapping : 1;/* bit 19:己方与资源区不重叠的补给区/RMUL补给区 */
    uint32_t ally_supply_zone_overlapping : 1;   /* bit 20:己方与资源区重叠的补给区 */
    uint32_t ally_assembly_zone : 1;             /* bit 21:己方装配增益点 */
    uint32_t enemy_assembly_zone : 1;            /* bit 22:对方装配增益点 */
    uint32_t center_buff_zone : 1;               /* bit 23:中心增益点(仅RMUL适用) */
    uint32_t enemy_fortress : 1;                 /* bit 24:对方堡垒增益点 */
    uint32_t enemy_outpost : 1;                  /* bit 25:对方前哨站增益点 */
    uint32_t ally_tunnel_highway_lower : 1;      /* bit 26:己方地形跨越增益点(隧道)(靠近己方一侧公路区下方) */
    uint32_t ally_tunnel_highway_middle : 1;     /* bit 27:己方地形跨越增益点(隧道)(靠近己方一侧公路区中间) */
    uint32_t ally_tunnel_highway_upper : 1;      /* bit 28:己方地形跨越增益点(隧道)(靠近己方一侧公路区上方) */
    uint32_t ally_tunnel_trapezoidal_lower : 1;  /* bit 29:己方地形跨越增益点(隧道)(靠近己方梯形高地较低处) */
    uint32_t ally_tunnel_trapezoidal_middle : 1; /* bit 30:己方地形跨越增益点(隧道)(靠近己方梯形高地较中间) */
    uint32_t ally_tunnel_trapezoidal_upper : 1;  /* bit 31:己方地形跨越增益点(隧道)(靠近己方梯形高地较高处) */

    uint8_t enemy_tunnel_highway_lower : 1;      /* bit 0:对方地形跨越增益点(隧道)(靠近对方公路一侧下方) */
    uint8_t enemy_tunnel_highway_middle : 1;     /* bit 1:对方地形跨越增益点(隧道)(靠近对方公路一侧中间) */
    uint8_t enemy_tunnel_highway_upper : 1;      /* bit 2:对方地形跨越增益点(隧道)(靠近对方公路一侧上方) */
    uint8_t enemy_tunnel_trapezoidal_lower : 1;  /* bit 3:对方地形跨越增益点(隧道)(靠近对方梯形高地较低处) */
    uint8_t enemy_tunnel_trapezoidal_middle : 1; /* bit 4:对方地形跨越增益点(隧道)(靠近对方梯形高地较中间) */
    uint8_t enemy_tunnel_trapezoidal_upper : 1;  /* bit 5:对方地形跨越增益点(隧道)(靠近对方梯形高地较高处) */
    uint8_t reserved_tunnel : 2;
} rfid_status_t;

typedef struct __packed
{
    uint8_t dart_launch_opening_status;          /* 当前飞镖发射站的状态: 1:关闭, 2:正在开启或者关闭中, 0:已经开启 */
    uint8_t reserved;                            /* 保留位 */
    uint16_t target_change_time;                 /* 切换击打目标时的比赛剩余时间,单位:秒,无/未切换动作,默认为0 */
    uint16_t latest_launch_cmd_time;             /* 最后一次操作手确定发射指令时的比赛剩余时间,单位:秒,初始值为0 */
} dart_client_cmd_t;

typedef struct __packed
{
    float hero_x;                                /* 己方英雄机器人位置x轴坐标,单位:m */
    float hero_y;                                /* 己方英雄机器人位置y轴坐标,单位:m */
    float engineer_x;                            /* 己方工程机器人位置x轴坐标,单位:m */
    float engineer_y;                            /* 己方工程机器人位置y轴坐标,单位:m */
    float standard_3_x;                          /* 己方3号步兵机器人位置x轴坐标,单位:m */
    float standard_3_y;                          /* 己方3号步兵机器人位置y轴坐标,单位:m */
    float standard_4_x;                          /* 己方4号步兵机器人位置x轴坐标,单位:m */
    float standard_4_y;                          /* 己方4号步兵机器人位置y轴坐标,单位:m */
    float reserved_1;                            /* 保留位 */
    float reserved_2;                            /* 保留位 */
} ground_robot_position_t;

/* 0x020C 雷达标记进度数据 */
typedef struct __packed
{
    uint16_t enemy_hero_mark : 1;          /* bit 0:对方1号英雄机器人易伤情况 */
    uint16_t enemy_engineer_mark : 1;      /* bit 1:对方2号工程机器人易伤情况 */
    uint16_t enemy_infantry_3_mark : 1;    /* bit 2:对方3号步兵机器人易伤情况 */
    uint16_t enemy_infantry_4_mark : 1;    /* bit 3:对方4号步兵机器人易伤情况 */
    uint16_t enemy_aerial_mark : 1;        /* bit 4:对方空中机器人特殊标识情况 */
    uint16_t enemy_sentry_mark : 1;        /* bit 5:对方哨兵机器人易伤情况 */
    uint16_t ally_hero_mark : 1;           /* bit 6:己方1号英雄机器人特殊标识情况 */
    uint16_t ally_engineer_mark : 1;       /* bit 7:己方2号工程机器人特殊标识情况 */
    uint16_t ally_infantry_3_mark : 1;     /* bit 8:己方3号步兵机器人特殊标识情况 */
    uint16_t ally_infantry_4_mark : 1;     /* bit 9:己方4号步兵机器人特殊标识情况 */
    uint16_t ally_aerial_mark : 1;         /* bit 10:己方空中机器人特殊标识情况 */
    uint16_t ally_sentry_mark : 1;         /* bit 11:己方哨兵机器人特殊标识情况 */
    uint16_t reserved : 4;                 /* bit 12-15:保留位 */
} radar_mark_data_t;

/* 0x020D 哨兵自主决策信息同步 */
typedef struct __packed
{
    uint32_t sentry_redeemed_projectile_allowance : 11; /* bit 0-10:除远程兑换外,哨兵机器人成功兑换的允许发弹量 */
    uint32_t sentry_remote_redeemed_projectile_times : 4;/* bit 11-14:哨兵机器人成功远程兑换允许发弹量的次数 */
    uint32_t sentry_remote_redeemed_HP_times : 4;       /* bit 15-18:哨兵机器人成功远程兑换血量的次数 */
    uint32_t can_confirm_free_revive : 1;               /* bit 19:哨兵机器人当前是否可以确认免费复活,可以确认免费复活时值为1,否则为0 */
    uint32_t can_redeem_immediate_revive : 1;           /* bit 20:哨兵机器人当前是否可以兑换立即复活,可以兑换立即复活时值为1,否则为0 */
    uint32_t cost_for_immediate_revive : 10;            /* bit 21-30:哨兵机器人当前若兑换立即复活需要花费的金币数 */
    uint32_t reserved_1 : 1;                            /* bit 31:保留 */

    uint16_t is_out_of_combat : 1;                       /* bit 0:哨兵当前是否处于脱战状态,处于脱战状态时为1,否则为0 */
    uint16_t remaining_redeemable_projectile_allowance : 11;/* bit 1-11:队伍17mm允许发弹量的剩余可兑换数 */
    uint16_t sentry_posture : 2;                         /* bit 12-13:哨兵当前姿态,1为进攻姿态,2为防御姿态,3为移动姿态 */
    uint16_t can_enter_activating_state : 1;             /* bit 14:己方能量机关是否能够进入正在激活状态,1为当前可激活 */
    uint16_t reserved_2 : 1;                             /* bit 15:保留位 */
} sentry_info_t;

/* 0x020E 雷达自主决策信息同步 */
typedef struct __packed
{
    uint8_t radar_double_damage_chance : 2;   /* bit 0-1:雷达是否拥有触发双倍易伤的机会,开局为0,至多为2 */
    uint8_t enemy_is_double_damaged : 1;      /* bit 2:对方是否正在被触发双倍易伤。0:对方未被触发双倍易伤, 1:对方正在被触发双倍易伤 */
    uint8_t ally_encryption_level : 2;        /* bit 3-4:己方加密等级(即对方干扰波难度等级),开局为1,最高为3 */
    uint8_t can_modify_password : 1;          /* bit 5:当前是否可以修改密钥,1为可修改 */
    uint8_t reserved : 2;                     /* bit 6-7:保留位 */
} radar_info_t;

typedef struct __packed
{
    uint16_t data_cmd_id;           /* 子内容 ID */
    uint16_t sender_id;             /* 发送者 ID */
    uint16_t receiver_id;           /* 接收者 ID */
    uint8_t user_data[112];         /* 内容数据段, x最大为112 */
} robot_interaction_data_t;

/* 图形结构 */
typedef struct __packed
{
    uint8_t figure_name[3];         /* 图形名,在图形删除、修改等操作中,作为索引 */
    uint32_t operate_type:3;        /* bit 0-2:图形操作。0:空操作, 1:增加, 2:修改, 3:删除 */
    uint32_t figure_type:3;         /* bit 3-5:图形类型。0:直线, 1:矩形, 2:正圆, 3:椭圆, 4:圆弧, 5:浮点数, 6:整型数, 7:字符 */
    uint32_t layer:4;               /* bit 6-9:图层数(0~9) */
    uint32_t color:4;               /* bit 10-13:颜色。0:红/蓝(己方颜色), 1:黄色, 2:绿色, 3:橙色, 4:紫红色, 5:粉色, 6:青色, 7:黑色, 8:白色 */
    uint32_t details_a:9;           /* bit 14-22:详见“表1-28 图形细节参数说明” */
    uint32_t details_b:9;           /* bit 23-31:详见“表1-28 图形细节参数说明” */
    uint32_t width:10;              /* bit 0-9:线宽,建议字体大小与线宽比例为10:1 */
    uint32_t start_x:11;            /* bit 10-20:起点/圆心x坐标 */
    uint32_t start_y:11;            /* bit 21-31:起点/圆心y坐标 */
    uint32_t details_c:10;          /* 根据绘制的图形不同,含义不同,详见“表1-28 图形细节参数说明” */
    uint32_t details_d:11;          /* 根据绘制的图形不同,含义不同,详见“表1-28 图形细节参数说明” */
    uint32_t details_e:11;          /* 根据绘制的图形不同,含义不同,详见“表1-28 图形细节参数说明” */
} interaction_figure_t;

/* UI字符 */
typedef struct __packed
{
    uint16_t data_id;                            /* 数据的内容 ID, 0x0110 */
    uint16_t tx_id;                              /* 发送者的ID */
    uint16_t rx_id;                              /* 接收者的ID */
    uint8_t Character_configuration[15];         /* 字符配置 */
    uint8_t Character[30];                       /* 字符 */
} graphic_data_struct_t;

typedef struct __packed
{
    float target_position_x;        /* 目标位置x轴坐标,单位m */
    float target_position_y;        /* 目标位置y轴坐标,单位m */
    uint8_t cmd_keyboard;           /* 云台手按下的键盘按键通用键值 */
    uint8_t target_robot_id;        /* 对方机器人 ID */
    uint16_t cmd_source;            /* 信息来源 ID */
} map_command_t;

/* 0x0120 哨兵自主决策指令 */
typedef struct __packed
{
    uint32_t confirm_resurrection : 1;                      /* bit 0:哨兵机器人是否确认复活。0表示哨兵机器人确认不复活, 1表示哨兵机器人确认复活 */
    uint32_t confirm_immediate_resurrection : 1;            /* bit 1:哨兵机器人是否确认兑换立即复活。0表示确认不兑换, 1表示确认兑换立即复活 */
    uint32_t redeem_projectile_allowance : 11;              /* bit 2-12:哨兵将要兑换的发弹量值 */
    uint32_t remote_redeem_projectile_request_times : 4;    /* bit 13-16:哨兵远程兑换发弹量的请求次数 */
    uint32_t remote_redeem_HP_request_times : 4;            /* bit 17-20:哨兵远程兑换血量的请求次数 */
    uint32_t sentry_posture_cmd : 2;                        /* bit 21-22:哨兵修改当前姿态指令,1为进攻姿态,2为防御姿态,3为移动姿态 */
    uint32_t confirm_activate_energy_mechanism : 1;         /* bit 23:哨兵机器人是否确认使能量机关进入正在激活状态,1为确认 */
    uint32_t reserved : 8;                                  /* bit 24-31:保留位 */
} sentry_cmd_t;

/* 0x0121 雷达自主决策指令 */
typedef struct __packed
{
    uint8_t radar_double_damage_cmd;  /* 雷达是否确认触发双倍易伤 */
    uint8_t password_cmd;             /* 密钥更新或验证指令, byte1为指令类型 */
    uint8_t password_1;               /* 密钥值 byte 2 */
    uint8_t password_2;               /* 密钥值 byte 3 */
    uint8_t password_3;               /* 密钥值 byte 4 */
    uint8_t password_4;               /* 密钥值 byte 5 */
    uint8_t password_5;               /* 密钥值 byte 6 */
    uint8_t password_6;               /* 密钥值 byte 7 */
} radar_cmd_t;

typedef struct __packed
{
    uint16_t hero_position_x;         /* 对方英雄机器人x位置坐标,单位:cm */
    uint16_t hero_position_y;         /* 对方英雄机器人y位置坐标,单位:cm */
    uint16_t engineer_position_x;     /* 对方工程机器人x位置坐标,单位:cm */
    uint16_t engineer_position_y;     /* 对方工程机器人y位置坐标,单位:cm */
    uint16_t infantry_3_position_x;   /* 对方3号步兵机器人x位置坐标,单位:cm */
    uint16_t infantry_3_position_y;   /* 对方3号步兵机器人y位置坐标,单位:cm */
    uint16_t infantry_4_position_x;   /* 对方4号步兵机器人x位置坐标,单位:cm */
    uint16_t infantry_4_position_y;   /* 对方4号步兵机器人y位置坐标,单位:cm */
    uint16_t reserved_1;              /* 对方6号空中机器人x位置坐标,单位:cm */
    uint16_t reserved_2;              /* 对方6号空中机器人y位置坐标,单位:cm */
    uint16_t sentry_position_x;       /* 对方哨兵机器人x位置坐标,单位:cm */
    uint16_t sentry_position_y;       /* 对方哨兵机器人y位置坐标,单位:cm */
} map_robot_data_t;

typedef struct __packed
{
    uint8_t intention;                /* 1:到目标点攻击, 2:到目标点防守, 3:移动到目标点 */
    uint16_t start_position_x;        /* 路径起点x轴坐标,单位:dm */
    uint16_t start_position_y;        /* 路径起点y轴坐标,单位:dm */
    int8_t delta_x[49];               /* 路径点x轴增量数组,单位:dm */
    int8_t delta_y[49];               /* 路径点y轴增量数组,单位:dm */
    uint16_t sender_id;               /* 发送者 ID */
} map_data_t;

typedef struct __packed
{
    uint16_t sender_id;               /* 发送者的ID */
    uint16_t receiver_id;             /* 接收者的ID */
    uint8_t user_data[30];            /* 字符 */
} custom_info_t;

/* 0x0306 自定义控制器键鼠操作 */
typedef struct __packed
{
    uint16_t key_value;               /* 键盘键值: bit 0-7:按键1键值, bit 8-15:按键2键值 */
    uint16_t x_position:12;           /* bit 0-11:鼠标X轴像素位置 */
    uint16_t mouse_left:4;            /* bit 12-15:鼠标左键状态 */
    uint16_t y_position:12;           /* bit 0-11:鼠标Y轴像素位置 */
    uint16_t mouse_right:4;           /* bit 12-15:鼠标右键状态 */
    uint16_t reserved;                /* 保留位 */
} custom_client_data_t;

typedef struct
{
    Offline_Check_t offline;
    game_status_t game_status;                           /* 内部包含：0x0001 比赛状态数据段 */
    game_result_t game_result;                           /* 内部包含：0x0002 比赛结果数据段 */
    game_robot_HP_t game_robot_HP;                       /* 内部包含：0x0003 所有机器人实时血量数据段 */

    referee_warning_t referee_warning;                   /* 内部包含：0x0104 裁判警告数据段 */
    dart_info_t dart_info;                               /* 内部包含：0x0105 飞镖发射相关数据段 */
    robot_status_t robot_status;                         /* 内部包含：0x0201 机器人性能体系数据段 */
    power_heat_data_t power_heat_data;                   /* 内部包含：0x0202 实时底盘缓冲能量和射击热量数据段 */
    robot_pos_t robot_pos;                               /* 内部包含：0x0203 机器人位置数据段 */
    buff_t buff;                                         /* 内部包含：0x0204 机器人增益和底盘能量数据段 */
    hurt_data_t hurt_data;                               /* 内部包含：0x0206 伤害状态数据段 */
    shoot_data_t shoot_data;                             /* 内部包含：0x0207 实时射击数据段 */
    projectile_allowance_t projectile_allowance;         /* 内部包含：0x0208 允许发弹量数据段 */
    rfid_status_t rfid_status;                           /* 内部包含：0x0209 机器人 RFID 模块状态数据段 */
    dart_client_cmd_t dart_client_cmd;                   /* 内部包含：0x020A 飞镖选手端指令数据段 */

    ground_robot_position_t ground_robot_position;       /* 内部包含：0x020B 地面机器人位置数据段 */
    radar_mark_data_t radar_mark_data;                   /* 内部包含：0x020C 雷达标记进度数据段 */
    sentry_info_t sentry_info;                           /* 内部包含：0x020D 哨兵自主决策信息同步段 */
    radar_info_t radar_info;                             /* 内部包含：0x020E 雷达自主决策信息同步段 */

    map_command_t map_command;                           /* 内部包含：0x0303 选手端小地图交互数据段 */
    event_data_t event_data;                             /* 内部包含：0x0101 场地事件数据段 */
    custom_info_t custom_info;                           /* 内部包含：0x0308 选手端小地图接收机器人数据段 */
} Referee_Data_t;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef frame_header;              /* 数据帧起始字节, 长度, 序号及 CRC8 */
        uint16_t read_cmd_id;
    } RX_Data_head;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_status_t game_status;
        uint16_t frame_tail;
    } RX_Data_game_status;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        robot_status_t robot_status;
        uint16_t frame_tail;
    } RX_Data_robot_status;

    uint8_t Data[255];

} ALL_RX_Data_T;

void Referee_System_Frame_Update(uint8_t *Buff, void *device_ptr, uint16_t Size);

void Referee_Send_KeyMouse(custom_client_data_t *control_data);

void Referee_Send_Data(uint16_t cmd_id, uint8_t *p_data, uint16_t len);

#endif