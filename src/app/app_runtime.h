#ifndef _app_runtime_h_
#define _app_runtime_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define app_runtime app_runtime_interface

extern const struct AppRuntimeInterface {
    void (*init)(void);
    void (*process)(void);
} app_runtime_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/** @brief 装配云台，条件就绪后自动运行 */
void app_runtime_init(void);

/** @brief 推进服务，处理 PA15 模式请求，显示实际工作状态 */
void app_runtime_process(void);

#endif
