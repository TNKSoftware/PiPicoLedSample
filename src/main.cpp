#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_NeoPixel.h>

#include "HidLampArray.h"
#include "LampArrayReports.h"

uint8_t const desc_lighting_report[] = {
    TUD_HID_REPORT_DESC_LIGHTING(0x01)
};

#define LAMP_PIN 23
#define LAMP_COUNT 1
#define NEO_PIXEL_TYPE (NEO_GRB + NEO_KHZ800)

// デバイスの物理的な大きさ
#define BOUND_WIDTH_MM 23
#define BOUND_HEIGHT_MM 53
#define BOUND_DEPTH_MM 2
// ランプが遅延するミリ秒
#define LAMP_UPDATE_LATENCY (0x04)

Adafruit_USBD_HID usb_hid;
Adafruit_NeoPixel neoPixelShield = Adafruit_NeoPixel(LAMP_COUNT, LAMP_PIN, NEO_PIXEL_TYPE);

uint16_t lastLampIdRequested;
bool is_autonomous, color_reset;
bool is_completed, require_update;

uint16_t OnGetReport(uint8_t report_id, hid_report_type_t report_type, uint8_t* data, uint16_t length);
void OnSetReport(uint8_t report_id, hid_report_type_t report_type, uint8_t const* data, uint16_t length);

void setup()
{
    TinyUSBDevice.setManufacturerDescriptor("TNK Software");
    TinyUSBDevice.setProductDescriptor("Pico Led Controller");

    if (!TinyUSBDevice.isInitialized()) {
        TinyUSBDevice.begin(0);
    }

    Serial.begin(115200);

    usb_hid.enableOutEndpoint(true);
    usb_hid.setPollInterval(2);
    usb_hid.setReportDescriptor(desc_lighting_report, sizeof(desc_lighting_report));
    usb_hid.setReportCallback(OnGetReport, OnSetReport);
    usb_hid.begin();
    
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    neoPixelShield.begin();
    neoPixelShield.clear();
	neoPixelShield.show();

    lastLampIdRequested = 0;
    is_autonomous = true;
    color_reset = false;
    is_completed = false;
    require_update = false;
}

void loop() 
{
#ifdef TINYUSB_NEED_POLLING_TASK
    TinyUSBDevice.task();
#endif

    if(is_completed == true && is_autonomous == false){
        // シリアル通信をしている状態だと即座に反応しないことがある
        if(require_update == true){
            neoPixelShield.show();
            color_reset = false;
        }
        require_update = false;
        is_completed = false;
    }else if(is_autonomous == true && color_reset == false){
        neoPixelShield.clear();
        neoPixelShield.show();
        color_reset = true;
    }
}

// USBがサスペンド状態になったときにTinyUSBから呼び出される
void tud_suspend_cb(bool remote_wakeup_en)
{
    // 消灯
    neoPixelShield.clear();
	neoPixelShield.show();

    __wfi();  // Pythonのlightsleep()と同じ
}

uint16_t SendLampArrayAttributesReport(LampArrayAttributesReport *report)
{
    // 物理情報を返す
    report->LampCount = LAMP_COUNT;
    report->BoundingBoxWidthInMicrometers = MILLIMETERS_TO_MICROMETERS(BOUND_WIDTH_MM);
    report->BoundingBoxHeightInMicrometers = MILLIMETERS_TO_MICROMETERS(BOUND_HEIGHT_MM);
    report->BoundingBoxDepthInMicrometers = MILLIMETERS_TO_MICROMETERS(BOUND_DEPTH_MM);
    report->LampArrayKind = LampArrayKindPeripheral;
    report->MinUpdateIntervalInMicroseconds = MILLISECONDS_TO_MICROSECONDS(33);

    return sizeof(LampArrayAttributesReport);
}

void UpdateLampAttributes(LampAttributesRequestReport *report) noexcept
{
    // 対象となるLampId(0～LampCount-1)の受信。無効なLampIdは0として処理する
    lastLampIdRequested = (report->LampId < LAMP_COUNT) ? report->LampId : 0;
}

uint16_t SendLampAttributesReport(LampAttributesResponseReport *report) noexcept
{
    report->Attributes.LampId = lastLampIdRequested;
    report->Attributes.PositionXInMicrometers  = MILLIMETERS_TO_MICROMETERS(BOUND_WIDTH_MM / 2);
    report->Attributes.PositionYInMicrometers  = MILLIMETERS_TO_MICROMETERS(48);
    report->Attributes.PositionZInMicrometers  = MILLIMETERS_TO_MICROMETERS(0);
    report->Attributes.UpdateLatencyInMicroseconds = MILLISECONDS_TO_MICROSECONDS(LAMP_UPDATE_LATENCY);
    report->Attributes.LampPurposes = LampPurposeAccent;
    report->Attributes.RedLevelCount = 0xFF;
    report->Attributes.GreenLevelCount = 0xFF;
    report->Attributes.BlueLevelCount = 0xFF;
    report->Attributes.IntensityLevelCount = 0x01;
    report->Attributes.IsProgrammable = LAMP_IS_PROGRAMMABLE;
    report->Attributes.InputBinding = 0x00;

    lastLampIdRequested++;
    if (lastLampIdRequested >= LAMP_COUNT) lastLampIdRequested = 0; // Reset

    return sizeof(LampAttributesResponseReport);
}

// デバイス設定の制御(AutonomousModeが無効ならデバイスでの制御はできず、有効ならデバイスでの変更が認められる)
void UpdateArrayControl(LampArrayControlReport *report) noexcept
{
    is_autonomous = !!report->AutonomousMode;
}

void setPixelColor(uint16_t id, uint32_t pxcolor)
{
    uint32_t old_color = neoPixelShield.getPixelColor(id);
    if(old_color != pxcolor){
        neoPixelShield.setPixelColor(id, pxcolor);
        require_update = true;
    }
}

// 複数のランプを一度に更新
void UpdateMultipleLamp(LampMultiUpdateReport *report) noexcept
{
    for (uint8_t i = 0; i < report->LampCount; i++){
        if (report->LampIds[i] < LAMP_COUNT) {
            LampArrayColor *c = &report->UpdateColors[i];
            uint32_t pxcolor = neoPixelShield.Color(c->RedChannel, c->GreenChannel, c->BlueChannel);
            setPixelColor(report->LampIds[i], pxcolor);
        }
    }

    // ホストから送られるデータが最後ならこのフラグが立ち、次に「LampArrayControlReport(AutonomousMode: enabled)」が送信されることになる
    if (report->LampUpdateFlags & LAMP_UPDATE_FLAG_UPDATE_COMPLETE) is_completed = true;
}

// 2つのIDの範囲内のランプを更新
void UpdateRangeLamp(LampRangeUpdateReport *report) noexcept
{
    if (report->LampIdStart >= 0 && report->LampIdStart < LAMP_COUNT && 
        report->LampIdEnd >= 0 && report->LampIdEnd < LAMP_COUNT && 
        report->LampIdStart <= report->LampIdEnd)
    {
        for (uint8_t i = report->LampIdStart; i <= report->LampIdEnd; i++) {
            uint32_t pxcolor = neoPixelShield.Color(report->UpdateColor.RedChannel, report->UpdateColor.GreenChannel, report->UpdateColor.BlueChannel);
            setPixelColor(i, pxcolor);
        }
    }

    // ホストから送られるデータが最後ならこのフラグが立ち、次に「LampArrayControlReport(AutonomousMode: enabled)」が送信されることになる
    if (report->LampUpdateFlags & LAMP_UPDATE_FLAG_UPDATE_COMPLETE) is_completed = true;
}

uint16_t OnGetReport(uint8_t report_id, hid_report_type_t report_type, uint8_t* data, uint16_t length) 
{
    uint16_t send_size = 0;

    switch (report_id){
    case LAMP_ARRAY_ATTRIBUTES: // 1: 機具の情報を返す。一番最初にホストから送られる。
        send_size = SendLampArrayAttributesReport((LampArrayAttributesReport*)data);
        break;

    case LAMP_ATTRIBUTES_RESPONSE: // 3: 要求されたIDに対応するランプの属性を返す
        send_size = SendLampAttributesReport((LampAttributesResponseReport*)data);
        break;
    }

    return send_size;
}

void OnSetReport(uint8_t report_id, hid_report_type_t report_type, uint8_t const* data, uint16_t length) 
{
    switch (report_id){
    case LAMP_ATTRIBUTES_REQUEST: // 2: LampIdの定義を返す
        UpdateLampAttributes((LampAttributesRequestReport*)data);
        break;

    // LAMP_ARRAY_CONTROLの後に送られる
    case LAMP_MULTI_UPDATE: // 4
        UpdateMultipleLamp((LampMultiUpdateReport*)data);
        break;
    case LAMP_RANGE_UPDATE: // 5
        UpdateRangeLamp((LampRangeUpdateReport*)data);
        break;

    case LAMP_ARRAY_CONTROL: // 6 : コントロールモードの変更要求
        UpdateArrayControl((LampArrayControlReport*)data);
        break;
    }
}