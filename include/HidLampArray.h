// Copyright (c) Microsoft Corporation.  All rights reserved.
// Licensed under the MIT License.

#pragma once

#define LAMP_UPDATE_FLAG_UPDATE_COMPLETE 1

#define LAMP_NOT_PROGRAMMABLE 0x00
#define LAMP_IS_PROGRAMMABLE 0x01

#define MILLISECONDS_TO_MICROSECONDS(x) (x * 1000)
#define MILLIMETERS_TO_MICROMETERS(x) (x * 1000)

enum LampPurposeKind
{
    LampPurposeControl = 1,        // 調整用
    LampPurposeAccent = 2,         // ユーザーへの情報がないランプ(ファンのLED、キーボードサイドの照明など)
    LampPurposeBranding = 4,       // 企業ロゴなどのブランディング
    LampPurposeStatus = 8,         // メール通知やCPU温度などの状態表示
    LampPurposeIllumination = 16,  // イルミネーション
    LampPurposePresentation = 32,  // アートワークや衣装など、ユーザーが直接目にする物
	// 0x10000以降はベンダー定義
};

enum LampArrayKind
{
    LampArrayKindKeyboard = 1,        // キーボード・キーパッド
    LampArrayKindMouse = 2,           // マウス
    LampArrayKindGameController = 3,  // ゲームコントローラー
    LampArrayKindPeripheral = 4,      // 周辺機器（スピーカー、マウスパッド）
    LampArrayKindScene = 5,           // 屋内照明
    LampArrayKindNotification = 6,    // 警告灯
    LampArrayKindChassis = 7,         // PCケース内パーツ(マザーボード、ファン)
    LampArrayKindWearable = 8,        // 身につける道具(靴、腕時計)
    LampArrayKindFurniture = 9,       // 家具
    LampArrayKindArt = 10,            // 美術品
	// 0x10000以降はベンダー定義
};

struct __attribute__ ((__packed__)) LampArrayColor
{
    uint8_t RedChannel;
    uint8_t GreenChannel;
    uint8_t BlueChannel;
    uint8_t IntensityChannel;
};

struct __attribute__ ((__packed__)) LampAttributes
{
    uint16_t LampId;                       // ID
    uint32_t PositionXInMicrometers;       // X位置
    uint32_t PositionYInMicrometers;       // Y位置
    uint32_t PositionZInMicrometers;       // Z位置
    uint32_t UpdateLatencyInMicroseconds;  // 遅延マイクロ秒
    uint32_t LampPurposes;                 // 使用目的(LampPurposeKind参照)
    uint8_t RedLevelCount;                 // 赤色の発光段階
    uint8_t GreenLevelCount;               // 緑色の発光段階
    uint8_t BlueLevelCount;                // 青色の発光段階
    uint8_t IntensityLevelCount;           // 明かりの強さの段階
    uint8_t IsProgrammable;                // プログラムによる制御が可能かどうか
    uint8_t InputBinding;                  // キーボードもしくはマウス用途であれば、関連付けるキーもしくはボタン
};