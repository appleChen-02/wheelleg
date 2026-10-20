#ifndef USB_TASK_H
#define USB_TASK_H

#include <stdbool.h>

#include "robot_param.h"
#include "usb_typdef.h"

extern void usb_task(void const * argument);
extern bool GetRlLowCmdData(ReceiveDataRlLowCmd_s * command);
extern bool IsRlLowCmdSessionEnabled(void);

#endif /* USB_TASK_H */
