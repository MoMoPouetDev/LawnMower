#ifndef _FSM_ENUM_H_
#define _FSM_ENUM_H_

#define PHASE_DOCK_INIT                                             0x00
#define PHASE_DOCK_WAITING_FOR_MOW                                  0x0F   //15

#define PHASE_OPERATIVE_INIT                                        0x10   //16
#define PHASE_OPERATIVE_WAITING_FOR_RETURNING_TO_BASE               0x1F   //31

#define PHASE_RETURN_TO_BASE_INIT                                   0x20   //32
#define PHASE_RETURN_TO_BASE_WAITING_DOCKING                        0x2F   //47

/* DECLARATION DES ETATS DU MODULE SUPERVISOR */
 typedef enum
 {
     S_SUP_DOCK_Init                            = PHASE_DOCK_INIT,
     S_SUP_DOCK_In_Charge                       = PHASE_DOCK_INIT + 1U,
     S_SUP_DOCK_Time_To_Mow                     = S_SUP_DOCK_In_Charge + 1U,
     S_SUP_DOCK_Waiting_For_Mow                 = PHASE_DOCK_WAITING_FOR_MOW,

     S_SUP_CALIBRATION_Init                              = PHASE_CALIBRATION_INIT,
     S_SUP_CALIBRATION_Waiting_For_Loading_Conso         = PHASE_CALIBRATION_WAITING_FOR_LOADING_CONSO,

     S_SUP_LOADING_CONSO_Init                            = PHASE_LOADING_CONSO_INIT,//32
     S_SUP_LOADING_CONSO_Sterile_Interface_Installation    = S_SUP_LOADING_CONSO_Init + 1U,//33
     S_SUP_LOADING_CONSO_Waiting_For_Detection           = S_SUP_LOADING_CONSO_Sterile_Interface_Installation + 1U,//34
     S_SUP_LOADING_CONSO_Rear_Module                     = S_SUP_LOADING_CONSO_Waiting_For_Detection + 1U,//35
     S_SUP_LOADING_CONSO_Central_Module                  = S_SUP_LOADING_CONSO_Rear_Module + 1U,//36
     S_SUP_LOADING_CONSO_Front_Module                    = S_SUP_LOADING_CONSO_Central_Module + 1U,//37
     S_SUP_LOADING_CONSO_Rear_Central_Modules            = S_SUP_LOADING_CONSO_Front_Module + 1U,//38
     S_SUP_LOADING_CONSO_Rear_Front_Modules              = S_SUP_LOADING_CONSO_Rear_Central_Modules + 1U,//39
     S_SUP_LOADING_CONSO_Central_Front_Modules           = S_SUP_LOADING_CONSO_Rear_Front_Modules + 1U,
     S_SUP_LOADING_CONSO_All                             = S_SUP_LOADING_CONSO_Central_Front_Modules + 1U,
     S_SUP_LOADING_CONSO_Conso_Validated                 = S_SUP_LOADING_CONSO_All + 1U,
     S_SUP_LOADING_CONSO_Waiting_For_Getting_Ready       = PHASE_LOADING_CONSO_WAITING_FOR_GETTING_READY,

     S_SUP_GETTING_READY_Init                            = PHASE_GETTING_READY_INIT,
     S_SUP_GETTING_READY_ACU                             = S_SUP_GETTING_READY_Init + 1U,
     S_SUP_GETTING_READY_Waiting_ACU                     = S_SUP_GETTING_READY_ACU + 1U,
     S_SUP_GETTING_READY_Waiting_For_Operative           = PHASE_GETTING_READY_WAITING_FOR_OPERATIVE,

     S_SUP_OPERATIVE_Init                                   = PHASE_OPERATIVE_INIT,//64
     S_SUP_OPERATIVE_No_Device                              = S_SUP_OPERATIVE_Init + 1U, //65
     S_SUP_OPERATIVE_Loading_Device                         = S_SUP_OPERATIVE_No_Device + 1U,//66
     S_SUP_OPERATIVE_Waiting_For_Switch_Pressed_lid_closed  = S_SUP_OPERATIVE_Loading_Device + 1U,//67
     S_SUP_OPERATIVE_KTG_Moving                             = S_SUP_OPERATIVE_Waiting_For_Switch_Pressed_lid_closed + 1U,//68
     S_SUP_OPERATIVE_MKT_Moving                             = S_SUP_OPERATIVE_KTG_Moving + 1U,
     S_SUP_OPERATIVE_Waiting_For_Track_Change               = S_SUP_OPERATIVE_MKT_Moving + 1U,
     S_SUP_OPERATIVE_Track_Change                           = S_SUP_OPERATIVE_Waiting_For_Track_Change + 1U,
     S_SUP_OPERATIVE_Sequential_Mode                        = S_SUP_OPERATIVE_Track_Change + 1U,
     S_SUP_OPERATIVE_Waiting_For_Unloading_Conso            = PHASE_OPERATIVE_WAITING_FOR_UNLOADING_CONSO,

     S_SUP_UNLOADING_CONSO_Init                                 = PHASE_UNLOADING_CONSO_INIT,//80
     S_SUP_UNLOADING_CONSO_Modules_Movement_Position_Zero_A     = S_SUP_UNLOADING_CONSO_Init + 1U,//81
     S_SUP_UNLOADING_CONSO_PS_Positioning                       = S_SUP_UNLOADING_CONSO_Modules_Movement_Position_Zero_A + 1U,//82
     S_SUP_UNLOADING_CONSO_STaR_Movement_Position_Unloading     = S_SUP_UNLOADING_CONSO_PS_Positioning + 1U,//83
     S_SUP_UNLOADING_CONSO_AC_unlocking                         = S_SUP_UNLOADING_CONSO_STaR_Movement_Position_Unloading + 1U,//84
     S_SUP_UNLOADING_CONSO_Modules_Movement_Position_Unloading  = S_SUP_UNLOADING_CONSO_AC_unlocking + 1U,//85
     S_SUP_UNLOADING_CONSO_Unloading_Conso                      = S_SUP_UNLOADING_CONSO_Modules_Movement_Position_Unloading + 1U,//86
     S_SUP_UNLOADING_CONSO_Waiting_For_Action                   = S_SUP_UNLOADING_CONSO_Unloading_Conso + 1U,//87
     S_SUP_UNLOADING_CONSO_Modules_Movement_Position_Zero_B     = S_SUP_UNLOADING_CONSO_Waiting_For_Action + 1U,//88
     S_SUP_UNLOADING_CONSO_Waiting_For_Shut_Down                = PHASE_UNLOADING_CONSO_WAITING_FOR_SHUT_DOWN,//95

     S_SUP_RESTART_Waiting_For_User_Decision             = PHASE_RESTART_INIT,
     S_SUP_RESTART_Init                                  = S_SUP_RESTART_Waiting_For_User_Decision + 1U,
     S_SUP_RESTART_Waiting_For_Operative                 = PHASE_RESTART_WAITING_FOR_OPERATIVE
 }S_MOWER_FSM_STATE;


 #endif /* _FSM_ENUM_H_ */
