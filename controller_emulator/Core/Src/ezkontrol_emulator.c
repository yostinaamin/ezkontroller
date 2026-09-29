/*
 * ezkontrol_emulator.c
 *
 * STM32F103 HAL module for EZkontrol motor controller emulator.
 *
 * This module emulates an EZkontrol controller over CAN for testing
 * the EZkontrol VCU board before connecting a real 72V motor controller.
 *
 * Behavior:
 *   1) At startup, repeatedly send handshake (0x55 pattern on 0x1801D0EF)
 *   2) Wait for VCU handshake reply (0xAA pattern on 0x0C01EFD0)
 *   3) Turn on HANDSHAKE LED when reply received
 *   4) Begin receiving VCU control commands (0x0C01EFD0, normal payload)
 *   5) Toggle COMMAND HEARTBEAT LED on each valid command
 *   6) Transmit simulated controller feedback:
 *      - 0x1801D0EF: voltage/current/speed/phase-current status
 *      - 0x1802D0EF: temperature/RUN/fault status
 *      - 0x180117EF: meter display voltage/current/speed
 *      - 0x180217EF: meter display throttle/gear/brake/temp/faults
 *   7) Implement command watchdog (250ms timeout)
 *
 * CubeMX assumption, 500 kbit/s:
 *   CAN clock 36 MHz, Prescaler=4, SJW=1TQ, BS1=15TQ, BS2=2TQ
 *   Enable CAN1 RX1 interrupt.
 *
 * Usage from main.c:
 *   after MX_GPIO_Init() and MX_CAN_Init():
 *       EZK_Emulator_Init();
 *
 *   inside while(1):
 *       EZK_Emulator_Task();
 *
 * GPIO Mapping (Blue Pill):
 *   PB0  = HANDSHAKE LED (ON after handshake)
 *   PB1  = COMMAND HEARTBEAT LED (toggle on each command)
 *   PB10 = RUN LED (ON when RUN command active)
 *   PB11 = WARNING LED (ON during comms failure/timeout)
 */

#include "main.h"
#include "ezkontrol_emulator.h"
#include <stdint.h>

extern CAN_HandleTypeDef hcan;

/* -------- EZkontrol extended CAN identifiers, MCU address 0xEF -------- */
#define EZK_ID_VCU_TO_MCU        0x0C01EFD0UL
#define EZK_ID_MCU_TO_VCU_1      0x1801D0EFUL
#define EZK_ID_MCU_TO_VCU_2      0x1802D0EFUL
#define EZK_ID_MCU_TO_METER_1    0x180117EFUL
#define EZK_ID_MCU_TO_METER_2    0x180217EFUL

/* Timing periods (milliseconds) */
#define HANDSHAKE_PERIOD_MS      250U
#define STATUS1_PERIOD_MS        50U
#define STATUS2_PERIOD_MS        50U
#define METER1_PERIOD_MS         100U
#define METER2_PERIOD_MS         100U
#define COMMAND_WATCHDOG_MS      250U

/* -------- LED GPIO Assignments -------- */
#define LED_HANDSHAKE_PORT       GPIOB
#define LED_HANDSHAKE_PIN        GPIO_PIN_0

#define LED_HEARTBEAT_PORT       GPIOB
#define LED_HEARTBEAT_PIN        GPIO_PIN_1

#define LED_RUN_PORT             GPIOB
#define LED_RUN_PIN              GPIO_PIN_10

#define LED_WARNING_PORT         GPIOB
#define LED_WARNING_PIN          GPIO_PIN_11

/* -------- Live Expressions / debug variables -------- */

/* Handshake state */
volatile uint32_t emu_handshake_tx_count = 0;
volatile uint32_t emu_handshake_reply_count = 0;
volatile uint8_t  emu_handshake_established = 0;

/* Command reception */
volatile uint32_t emu_command_rx_count = 0;
volatile uint32_t emu_last_command_tick = 0;
volatile uint32_t emu_command_timeout_count = 0;

/* Received VCU command fields */
volatile uint8_t  emu_cmd_run = 0;
volatile uint8_t  emu_cmd_speed_mode = 0;
volatile int16_t  emu_cmd_target_phase_current_0p1A = 0;
volatile int16_t  emu_cmd_target_speed_rpm = 0;
volatile uint8_t  emu_cmd_life = 0;

/* Transmission counters */
volatile uint32_t emu_status1_tx_count = 0;
volatile uint32_t emu_status2_tx_count = 0;
volatile uint32_t emu_meter1_tx_count = 0;
volatile uint32_t emu_meter2_tx_count = 0;

/* Simulated status feedback: 0x1801D0EF (MCU->VCU Message I) */
volatile uint16_t emu_bus_voltage_0p1V = 720;     /* 72.0 V default */
volatile int16_t  emu_bus_current_0p1A = 0;       /* 0.0 A default */
volatile int16_t  emu_phase_current_0p1A = 0;     /* 0.0 A default, follows command */
volatile int16_t  emu_speed_rpm = 0;              /* 0 rpm default, follows command */

/* Simulated status feedback: 0x1802D0EF (MCU->VCU Message II) */
volatile int16_t  emu_controller_temp_C = 25;
volatile int16_t  emu_motor_temp_C = 25;
volatile uint8_t  emu_controller_running = 0;
volatile uint8_t  emu_controller_speed_mode = 0;

/* Meter 1 (0x180117EF) - not currently exposed for manual edit */
/* Meter 2 (0x180217EF) - analog control visualization */
volatile uint8_t  emu_throttle_percent = 0;
volatile uint8_t  emu_gear_raw = 0;       /* 0-7: NO, R, N, D1, D2, D3, S, P */
volatile uint8_t  emu_brake_active = 0;
volatile uint8_t  emu_operation_mode_raw = 0;  /* 0=Stop, 1=Drive, 2=Cruise, 3=EBS, 4=Hold */
volatile uint8_t  emu_dc_contactor_on = 0;

volatile uint8_t  emu_meter_error_byte4 = 0;
volatile uint8_t  emu_meter_error_byte5 = 0;
volatile uint8_t  emu_meter_error_byte6 = 0;
volatile uint8_t  emu_meter_life = 0;

/* Error tracking */
volatile uint32_t emu_can_hal_error = 0;
volatile uint32_t emu_bad_dlc_count = 0;

/* Diagnostics: CAN register state */
volatile uint32_t emu_can_esr = 0;      /* Error Status Register */
volatile uint32_t emu_can_tsr = 0;      /* Transmit Status Register */
volatile uint32_t emu_can_rf1r = 0;     /* Receive FIFO 1 Register */
volatile uint8_t  emu_can_rec = 0;      /* Receive Error Counter */
volatile uint8_t  emu_can_tec = 0;      /* Transmit Error Counter */

/* Diagnostics: TX operations */
volatile uint32_t emu_tx_attempt_count = 0;
volatile uint32_t emu_tx_hal_ok_count = 0;
volatile uint32_t emu_tx_hal_busy_count = 0;
volatile uint32_t emu_tx_hal_error_count = 0;

/* Diagnostics: Initialization status */
volatile uint32_t emu_init_filter_result = 0;
volatile uint32_t emu_init_start_result = 0;
volatile uint32_t emu_init_notify_result = 0;

/* Diagnostics: Main loop alive counter */
volatile uint32_t emu_main_loop_alive = 0;

/* -------- Internal state -------- */
static CAN_RxHeaderTypeDef rxh;
static CAN_TxHeaderTypeDef txh;
static uint8_t rxd[8];
static uint32_t tx_mailbox;
static uint8_t life_counter = 0;

/* Last transmit times for periodic messages */
static uint32_t last_handshake_tx = 0;
static uint32_t last_status1_tx = 0;
static uint32_t last_status2_tx = 0;
static uint32_t last_meter1_tx = 0;
static uint32_t last_meter2_tx = 0;

static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* Encode phase current: 0.1 A/bit, offset -3200A
 * raw = current_0p1A + 32000 */
static uint16_t encode_phase_current(int32_t current_0p1A)
{
    int32_t raw = current_0p1A + 32000;
    if (raw < 0) raw = 0;
    if (raw > 64000) raw = 64000;
    return (uint16_t)raw;
}

/* Encode speed: 1 rpm/bit, offset -32000rpm
 * raw = rpm + 32000 */
static uint16_t encode_speed(int32_t rpm)
{
    int32_t raw = rpm + 32000;
    if (raw < 0) raw = 0;
    if (raw > 64000) raw = 64000;
    return (uint16_t)raw;
}

/* Encode bus voltage: 0.1 V/bit, unsigned 16-bit little-endian */
static uint16_t encode_bus_voltage(uint16_t voltage_0p1V)
{
    return voltage_0p1V;
}

/* Check if all 8 bytes are a specific value */
static uint8_t frame_is_8x(uint8_t value)
{
    uint8_t i;
    if (rxh.DLC != 8) return 0;
    for (i = 0; i < 8; i++)
        if (rxd[i] != value) return 0;
    return 1;
}

static void led_handshake_on(void)
{
    HAL_GPIO_WritePin(LED_HANDSHAKE_PORT, LED_HANDSHAKE_PIN, GPIO_PIN_SET);
}

static void led_heartbeat_toggle(void)
{
    HAL_GPIO_TogglePin(LED_HEARTBEAT_PORT, LED_HEARTBEAT_PIN);
}

static void led_run_set(uint8_t state)
{
    HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void led_warning_set(uint8_t state)
{
    HAL_GPIO_WritePin(LED_WARNING_PORT, LED_WARNING_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Send handshake pattern: 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 */
static void send_handshake(void)
{
    uint8_t tx[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    HAL_StatusTypeDef result;

    txh.ExtId = EZK_ID_MCU_TO_VCU_1;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    emu_tx_attempt_count++;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        result = HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);
        if (result == HAL_OK)
        {
            emu_tx_hal_ok_count++;
            emu_handshake_tx_count++;
        }
        else if (result == HAL_BUSY)
        {
            emu_tx_hal_busy_count++;
        }
        else
        {
            emu_tx_hal_error_count++;
        }
    }
}

/* Send MCU->VCU Message I: voltage, bus current, phase current, speed */
static void send_status1(void)
{
    uint8_t tx[8];
    uint16_t voltage_raw = encode_bus_voltage(emu_bus_voltage_0p1V);
    uint16_t bus_current_raw = (uint16_t)((int32_t)emu_bus_current_0p1A + 32000);
    uint16_t phase_current_raw = encode_phase_current(emu_phase_current_0p1A);
    uint16_t speed_raw = encode_speed(emu_speed_rpm);

    tx[0] = (uint8_t)(voltage_raw & 0xFF);
    tx[1] = (uint8_t)(voltage_raw >> 8);

    tx[2] = (uint8_t)(bus_current_raw & 0xFF);
    tx[3] = (uint8_t)(bus_current_raw >> 8);

    tx[4] = (uint8_t)(phase_current_raw & 0xFF);
    tx[5] = (uint8_t)(phase_current_raw >> 8);

    tx[6] = (uint8_t)(speed_raw & 0xFF);
    tx[7] = (uint8_t)(speed_raw >> 8);

    txh.ExtId = EZK_ID_MCU_TO_VCU_1;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);

    emu_status1_tx_count++;
}

/* Send MCU->VCU Message II: temperatures, RUN state, faults, life counter */
static void send_status2(void)
{
    uint8_t tx[8];

    tx[0] = (uint8_t)(emu_controller_temp_C + 40);
    tx[1] = (uint8_t)(emu_motor_temp_C + 40);

    tx[2] = 0;
    if (emu_controller_running) tx[2] |= (1U << 0);
    if (emu_controller_speed_mode) tx[2] |= (1U << 1);

    tx[3] = 0;  /* Reserved byte 3 */
    tx[4] = 0;  /* Error byte 4 (fault bits) */
    tx[5] = 0;  /* Error byte 5 (fault bits) */
    tx[6] = 0;  /* Error byte 6 (fault bits) */
    tx[7] = life_counter++;

    txh.ExtId = EZK_ID_MCU_TO_VCU_2;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);

    emu_status2_tx_count++;
}

/* Send MCU->Meter Message I: voltage, bus current, phase current, speed */
static void send_meter1(void)
{
    uint8_t tx[8];
    uint16_t voltage_raw = encode_bus_voltage(emu_bus_voltage_0p1V);
    uint16_t bus_current_raw = (uint16_t)((int32_t)emu_bus_current_0p1A + 32000);
    uint16_t phase_current_raw = encode_phase_current(emu_phase_current_0p1A);
    uint16_t speed_raw = encode_speed(emu_speed_rpm);

    tx[0] = (uint8_t)(voltage_raw & 0xFF);
    tx[1] = (uint8_t)(voltage_raw >> 8);

    tx[2] = (uint8_t)(bus_current_raw & 0xFF);
    tx[3] = (uint8_t)(bus_current_raw >> 8);

    tx[4] = (uint8_t)(phase_current_raw & 0xFF);
    tx[5] = (uint8_t)(phase_current_raw >> 8);

    tx[6] = (uint8_t)(speed_raw & 0xFF);
    tx[7] = (uint8_t)(speed_raw >> 8);

    txh.ExtId = EZK_ID_MCU_TO_METER_1;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);

    emu_meter1_tx_count++;
}

/* Send MCU->Meter Message II: throttle, gear, brake, temp, faults, DC contactor */
static void send_meter2(void)
{
    uint8_t tx[8];

    /* Byte 0: Controller Temperature (1 C/bit, offset -40 C) */
    tx[0] = (uint8_t)(emu_controller_temp_C + 40);

    /* Byte 1: Motor Temperature (1 C/bit, offset -40 C) */
    tx[1] = (uint8_t)(emu_motor_temp_C + 40);

    /* Byte 2: Accelerator Opening (0..100 %) */
    tx[2] = emu_throttle_percent;

    /* Byte 3: Gear (bits 0-2), Brake (bit 3), Operation Mode (bits 4-6), DC Contactor (bit 7) */
    tx[3] = 0;
    tx[3] |= (emu_gear_raw & 0x07);
    tx[3] |= (emu_brake_active ? (1U << 3) : 0);
    tx[3] |= ((emu_operation_mode_raw & 0x07) << 4);
    tx[3] |= (emu_dc_contactor_on ? (1U << 7) : 0);

    /* Bytes 4-6: Fault bytes */
    tx[4] = emu_meter_error_byte4;
    tx[5] = emu_meter_error_byte5;
    tx[6] = emu_meter_error_byte6;

    /* Byte 7: Life signal */
    tx[7] = emu_meter_life;

    txh.ExtId = EZK_ID_MCU_TO_METER_2;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);

    emu_meter2_tx_count++;
}

/* Process received VCU handshake reply (0xAA pattern) */
static void parse_handshake_reply(void)
{
    emu_handshake_reply_count++;
    emu_handshake_established = 1;
    led_handshake_on();
}

/* Process received VCU command on 0x0C01EFD0 (normal, not handshake) */
static void parse_vcu_command(void)
{
    uint16_t raw;

    emu_command_rx_count++;
    emu_last_command_tick = HAL_GetTick();
    led_heartbeat_toggle();

    /* Bytes 0-1: target phase current (0.1 A/bit, offset -3200A) */
    raw = u16le(&rxd[0]);
    emu_cmd_target_phase_current_0p1A = (int16_t)((int32_t)raw - 32000);

    /* Bytes 2-3: target speed (1 rpm/bit, offset -32000rpm) */
    raw = u16le(&rxd[2]);
    emu_cmd_target_speed_rpm = (int16_t)((int32_t)raw - 32000);

    /* Byte 4: RUN/HALT (bit 0), speed/torque mode (bit 1) */
    emu_cmd_run = (rxd[4] >> 0) & 0x01;
    emu_cmd_speed_mode = (rxd[4] >> 1) & 0x01;

    /* Bytes 5-6: reserved */

    /* Byte 7: VCU life counter */
    emu_cmd_life = rxd[7];

    /* Update LED states based on command */
    led_run_set(emu_cmd_run);

    /* Update simulated feedback to follow commands */
    if (emu_cmd_run)
    {
        emu_phase_current_0p1A = emu_cmd_target_phase_current_0p1A;
        emu_speed_rpm = emu_cmd_target_speed_rpm;
        emu_controller_running = 1;
    }
    else
    {
        emu_phase_current_0p1A = 0;
        emu_speed_rpm = 0;
        emu_controller_running = 0;
    }

    emu_controller_speed_mode = emu_cmd_speed_mode;
}

/* CAN RX FIFO1 callback - receives VCU commands and handshake replies */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan)
{
    if (phcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxh, rxd) != HAL_OK)
        return;

    if (rxh.IDE != CAN_ID_EXT) return;

    /* Only process messages from VCU to emulator */
    if (rxh.ExtId != EZK_ID_VCU_TO_MCU) return;

    if (rxh.DLC != 8)
    {
        emu_bad_dlc_count++;
        return;
    }

    /* Distinguish handshake reply (all 0xAA) from normal command */
    if (frame_is_8x(0xAA))
    {
        parse_handshake_reply();
    }
    else
    {
        parse_vcu_command();
    }
}

void EZK_Emulator_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* Configure pass-all extended CAN filter on FIFO1 */
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0;
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0;
    f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.SlaveStartFilterBank = 14;

    /* Record initialization results */
    emu_init_filter_result = HAL_CAN_ConfigFilter(&hcan, &f);
    if (emu_init_filter_result != HAL_OK)
        Error_Handler();

    emu_init_start_result = HAL_CAN_Start(&hcan);
    if (emu_init_start_result != HAL_OK)
        Error_Handler();

    emu_init_notify_result = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (emu_init_notify_result != HAL_OK)
        Error_Handler();

    /* Initialize LED state */
    led_warning_set(1);  /* Turn on WARNING LED initially */
    led_run_set(0);
    led_heartbeat_toggle();

    last_handshake_tx = HAL_GetTick();
    last_status1_tx = HAL_GetTick();
    last_status2_tx = HAL_GetTick();
    last_meter1_tx = HAL_GetTick();
    last_meter2_tx = HAL_GetTick();
    emu_last_command_tick = HAL_GetTick();
}

void EZK_Emulator_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t time_since_command;

    emu_main_loop_alive++;
    emu_can_hal_error = HAL_CAN_GetError(&hcan);

    /* Capture CAN register state for diagnostics */
    if (hcan.Instance == CAN1)
    {
        emu_can_esr = hcan.Instance->ESR;
        emu_can_tsr = hcan.Instance->TSR;
        emu_can_rf1r = hcan.Instance->RF1R;
        emu_can_rec = (emu_can_esr >> 24) & 0xFF;
        emu_can_tec = (emu_can_esr >> 16) & 0xFF;
    }

    /* Phase 1: Before handshake - send 0x55 pattern periodically */
    if (!emu_handshake_established)
    {
        if ((uint32_t)(now - last_handshake_tx) >= HANDSHAKE_PERIOD_MS)
        {
            send_handshake();
            last_handshake_tx = now;
        }
        return;
    }

    /* Phase 2: After handshake - send status messages and handle watchdog */

    /* Check for command timeout */
    time_since_command = (uint32_t)(now - emu_last_command_tick);
    if (time_since_command >= COMMAND_WATCHDOG_MS)
    {
        /* Timeout: safe halt and turn on warning LED */
        if (emu_cmd_run != 0)
        {
            emu_command_timeout_count++;
        }
        emu_cmd_run = 0;
        emu_phase_current_0p1A = 0;
        emu_speed_rpm = 0;
        emu_controller_running = 0;
        emu_operation_mode_raw = 0;  /* Stop mode */
        led_warning_set(1);
        led_run_set(0);
    }
    else
    {
        /* Communications healthy */
        led_warning_set(0);
    }

    /* Send MCU->VCU Message I (status 1) */
    if ((uint32_t)(now - last_status1_tx) >= STATUS1_PERIOD_MS)
    {
        send_status1();
        last_status1_tx = now;
    }

    /* Send MCU->VCU Message II (status 2) */
    if ((uint32_t)(now - last_status2_tx) >= STATUS2_PERIOD_MS)
    {
        send_status2();
        last_status2_tx = now;
    }

    /* Send MCU->Meter Message I (meter 1) */
    if ((uint32_t)(now - last_meter1_tx) >= METER1_PERIOD_MS)
    {
        send_meter1();
        last_meter1_tx = now;
    }

    /* Send MCU->Meter Message II (meter 2) */
    if ((uint32_t)(now - last_meter2_tx) >= METER2_PERIOD_MS)
    {
        send_meter2();
        emu_meter_life++;
        last_meter2_tx = now;
    }
}
