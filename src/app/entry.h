#ifndef _app_entry_h_
#define _app_entry_h_

/** @file entry.h
 * @brief CubeMX 应用入口；业务只通过 app_runtime 调用 service
 */

#include "app_runtime.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/** @brief 初始化云台 */
static inline void entry_init(void) {
    app_runtime.init();
}

/** @brief 主循环非阻塞推进 */
static inline void entry_loop(void) {
    app_runtime.process();
}

#endif
