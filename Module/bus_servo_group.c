#include "bus_servo_group.h"

#include <string.h>

enum { PH_REQUEST = 0, PH_WAIT };

static void advance(BusServoGroup *g) {
  g->idx++;
  if (g->idx >= g->n) {
    g->idx = 0;
    g->rounds++;
  }
  g->phase = PH_REQUEST;
}

void BusServoGroup_Init(BusServoGroup *g, BusServoPort port, const uint8_t *ids, uint8_t n,
                        uint16_t timeout) {
  if (g == NULL || ids == NULL) {
    return;
  }
  memset(g, 0, sizeof(*g));
  g->port = port;
  g->n = (n > BUS_SERVO_GROUP_MAX) ? BUS_SERVO_GROUP_MAX : n;
  g->timeout_ms = (timeout == 0u) ? 5u : timeout;
  g->mode = GROUP_IDLE;
  for (uint8_t i = 0; i < g->n; i++) {
    BusServo_Init(&g->servo[i], port, ids[i]);
    BusServo_SetTimeout(&g->servo[i], g->timeout_ms);
  }
}

void BusServoGroup_SetPeriod(BusServoGroup *g, uint16_t ms) {
  if (g != NULL) {
    g->period_ms = ms;
  }
}

void BusServoGroup_StartSweep(BusServoGroup *g, uint32_t now_ms) {
  if (g == NULL || g->n == 0u) {
    return;
  }
  g->mode = GROUP_SWEEP;
  g->idx = 0;
  g->phase = PH_REQUEST;
  g->next_start_ms = now_ms;
  g->rounds = 0;
  g->timeouts = 0;
}

void BusServoGroup_ReleaseAll(BusServoGroup *g, uint32_t now_ms) {
  (void)now_ms;
  if (g == NULL || g->n == 0u) {
    return;
  }
  g->mode = GROUP_RELEASE;
  g->idx = 0;
  g->phase = PH_REQUEST;
}

void BusServoGroup_Poll(BusServoGroup *g, uint32_t now_ms) {
  if (g == NULL || g->n == 0u) {
    return;
  }

  switch (g->mode) {
  case GROUP_SWEEP:
    if (g->phase == PH_REQUEST) {
      /* 每轮的第一条要等周期到点；背靠背（period=0）直接发。
       * next_start_ms 是「下一轮允许开始的时间」，按开始到开始计时，采样间隔才均匀。 */
      if (g->idx == 0u && g->period_ms > 0u) {
        if ((int32_t)(now_ms - g->next_start_ms) < 0) {
          return;
        }
        g->next_start_ms = now_ms + g->period_ms;
      }
      if (BusServo_RequestAngle(&g->servo[g->idx])) {
        g->phase = PH_WAIT;
      }
      return;
    }
    /* PH_WAIT */
    BusServo_Poll(&g->servo[g->idx]);
    switch (BusServo_GetState(&g->servo[g->idx])) {
    case BUS_SERVO_DONE: {
      uint16_t pwm = 0;
      if (BusServo_GetAngle(&g->servo[g->idx], &pwm)) {
        g->pos[g->idx] = pwm;
        g->pos_valid[g->idx] = true;
      }
      advance(g);
      break;
    }
    case BUS_SERVO_TIMEOUT:
      g->timeouts++;
      advance(g);
      break;
    default:
      break;
    }
    return;

  case GROUP_RELEASE:
    if (g->phase == PH_REQUEST) {
      if (BusServo_Release(&g->servo[g->idx])) {
        g->phase = PH_WAIT;
      }
      return;
    }
    BusServo_Poll(&g->servo[g->idx]);
    switch (BusServo_GetState(&g->servo[g->idx])) {
    case BUS_SERVO_DONE:
    case BUS_SERVO_TIMEOUT: {
      if (BusServo_GetState(&g->servo[g->idx]) == BUS_SERVO_TIMEOUT) {
        g->timeouts++;
      }
      g->idx++;
      if (g->idx >= g->n) {
        g->idx = 0;
        g->mode = GROUP_DONE;
      }
      g->phase = PH_REQUEST;
      break;
    }
    default:
      break;
    }
    return;

  case GROUP_IDLE:
  case GROUP_DONE:
  default:
    return;
  }
}

bool BusServoGroup_GetPos(const BusServoGroup *g, uint8_t idx, uint16_t *pwm) {
  if (g == NULL || pwm == NULL || idx >= g->n || !g->pos_valid[idx]) {
    return false;
  }
  *pwm = g->pos[idx];
  return true;
}

BusServoGroupMode BusServoGroup_GetMode(const BusServoGroup *g) {
  return (g == NULL) ? GROUP_IDLE : g->mode;
}

uint32_t BusServoGroup_GetRounds(const BusServoGroup *g) {
  return (g == NULL) ? 0u : g->rounds;
}

uint16_t BusServoGroup_GetTimeouts(const BusServoGroup *g) {
  return (g == NULL) ? 0u : g->timeouts;
}
