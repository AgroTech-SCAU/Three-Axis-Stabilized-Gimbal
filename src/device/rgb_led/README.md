# RGB LED 接口

## 分层

- `rgb_led.*` 绑定实例并提供统一接口
- `ws2812_rgb_led.*` 管理 GRB 颜色缓存与时序编码
- `assemble_rgb()` 绑定项目缓冲和 SPI6 端口
- `user_io` 决定状态颜色，设备层不处理运行模式

## 常用接口

| 接口 | 含义 |
|---|---|
| `rgb_led_set_instance()` | 初始化前绑定设备实例 |
| `rgb_led.init()` | 配置端口、像素数、缓存与异步模式 |
| `rgb_led.set_rgb()` / `fill()` | 修改颜色缓存，不发送 |
| `rgb_led.show()` | 编码并提交当前颜色 |
| `rgb_led_write_complete()` | 释放异步发送缓存占用 |

`rgb_led.xxx` 与 `rgb_led_xxx()` 包装函数访问同一实例

```c
typedef struct {
    bool (*write)(const uint8_t* data, uint32_t len);
} RgbLedPortOps;
```

`write` 接收已编码数据，返回 true 表示接受提交，不代表 DMA 完成或实物颜色确认

## 项目配置

| 项目 | 当前值 |
|---|---|
| 像素数 | 8 |
| 色序 | GRB，最高位先发 |
| 0 / 1 编码 | `0x60 / 0x78` |
| SPI6 | 6 MHz、8 bit、MSB、CPOL LOW、CPHA 2EDGE |
| 引脚与 DMA | PA7 MOSI、PA5 SCK、BDMA Channel0 |
| 发送缓冲 | `.ram_d3`、32 字节对齐 |
| 数据与帧尾 | `8 * 24 + 240 = 432` 字节 |

6 MHz 下 240 字节尾零主动保持 320 μs 低电平，不依赖关闭 SPI 后的引脚电平

`ws2812_rgb_led_make_config()` 的通用默认仍为 8 像素、80 字节尾零和同步模式，当前项目在装配时覆盖为 240 字节尾零与异步模式

## 异步约束

- 同步模式要求 `write` 返回前完成对发送缓存的读取
- 异步模式允许返回后继续读取，DMA 结束前 `show()` 返回 `RGB_LED_STATUS_BUSY`
- `fill()` 只改颜色缓存，不覆盖在途编码缓冲
- 成功完成与错误回调都须释放缓存占用，错误不能记为成功完成
- 当前只有一份发送缓冲，不支持连续排队多帧
- 上层使用返回值和 `spi6_get_tx_stats()` 判断提交、完成、拒绝与异步错误
- WS2812 没有颜色回读，目标与成功传输计数都不能确认实物颜色

状态指示见工程根目录 [README](../../../README.md)
