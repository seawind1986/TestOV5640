#ifndef OV5640_H
#define OV5640_H

#include <stdint.h>

#define OV5640_GEAR_COUNT 8U

int OV5640_SensorInit(void);
void OV5640_ApplyView(void);
/* 按正点原子 RGB565 例程：传感器把整幅缩放到屏幕大小。 */
void OV5640_RgbLcdPreview(uint16_t width, uint16_t height);
/* 0：不写传感器。非 0：把起点和范围写进传感器。 */
extern volatile uint8_t ov5640_win_write;
/* x、y 是阵列上的起点，宽高是开窗范围。坐标会收进阵列并按偶数对齐。 */
void OV5640_SetWindowPos(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
/* x、y 以摄像头画面中心为原点，向右、向下为正。开窗中心放在这个点上，再按同一比例缩进 LCD。 */
void OV5640_ShowOnLcd(int16_t x, int16_t y, uint16_t width, uint16_t height);
/* 缩进屏幕后的位置和大小。比屏幕瘦时左右留黑，比屏幕扁时上下留黑。 */
void OV5640_GetView(uint16_t *x, uint16_t *y, uint16_t *width, uint16_t *height);
/* 传感器实际吐出的宽高。缩小由传感器做，放大时这里仍是开窗原尺寸。 */
void OV5640_GetCapture(uint16_t *width, uint16_t *height);
/* 当前传感器开窗在阵列上的起点和宽高。 */
void OV5640_GetWindow(uint16_t *x, uint16_t *y, uint16_t *width, uint16_t *height);
/* 1：输出 1:1 原尺寸，不放大填屏。转速模式裁窗用。 */
void OV5640_SetNativeOutput(uint8_t on);
/* 按阵列绝对坐标开窗，1:1 输出，并把 src 记下以免后续 RgbLcdPreview 又居中。 */
void OV5640_SetNativeWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
/* 0：软件掉电停流。非 0：恢复出流（0x3008 = 0x02）。 */
void OV5640_SetStream(uint8_t on);
/* 整帧快门，单位微秒。只改曝光，不改增益。长过当前帧时把帧周期加长。 */
void OV5640_SetExposure(uint32_t shutter_us);
/* 10 位增益，0x350A 的低 2 位和 0x350B。只改增益，不改曝光。 */
void OV5640_SetGain(uint16_t gain);
/* 0：RGB565，给 LCD。1：JPEG。方向键现在只改裁剪，不触发拍照。 */
void OV5640_SetJpeg(uint8_t on);
void OV5640_GetSize(uint16_t *width, uint16_t *height);
int OV5640_Zoom(int direction);
int OV5640_Pan(int direction);
void OV5640_FocusLoad(void);
void OV5640_FocusSweep(void);
/* 0 没写入或写入失败，1 松开已确认，2 对焦已完成，3 固件已写入但命令未确认。 */
uint8_t OV5640_FocusState(void);
/* bit7 是以太网 PHY 复位，高电平为退出复位。新板这根线直连 PH5。 */
void OV5640_BoardIo(uint8_t bit, uint8_t high);

#endif
