#include "app_sr.h"
#include "app_mic.h"
#include "sdkconfig.h"

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "model_path.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <cstring>

static const char *TAG = "app_sr";

typedef struct {
    app_sr_event_t evt;
    int command_id;
} sr_queue_item_t;

static MEMS_MIC *s_mic = nullptr;
static const esp_afe_sr_iface_t *s_afe_handle = nullptr;
static esp_afe_sr_data_t *s_afe_data = nullptr;
static srmodel_list_t *s_models = nullptr;
static volatile bool s_task_run = false;
static QueueHandle_t s_evt_queue = nullptr;
static app_sr_event_cb_t s_user_cb = nullptr;

static void sr_emit_event(app_sr_event_t evt, int command_id, const char *cmd_str) {
  sr_queue_item_t item = {.evt = evt, .command_id = command_id};
  if (s_evt_queue) {
    xQueueSend(s_evt_queue, &item, 0);
  }
  if (s_user_cb) {
    s_user_cb(evt, command_id, cmd_str);
  }
}

static void sr_setup_commands(esp_mn_iface_t *multinet, model_iface_data_t *model_data) {
  esp_mn_commands_alloc(multinet, model_data);
  esp_mn_commands_clear();
  esp_mn_commands_add(1, "jiu ming");
  esp_mn_commands_add(2, "bao jing");
  esp_mn_commands_add(3, "liu yan");
  esp_mn_commands_update();
  multinet->print_active_speech_commands(model_data);
}

static void sr_feed_task(void *arg) {
  (void)arg;
  int chunk = s_afe_handle->get_feed_chunksize(s_afe_data);
  int nch = s_afe_handle->get_feed_channel_num(s_afe_data);
  int16_t *buf = (int16_t *)malloc(chunk * nch * sizeof(int16_t));
  if (!buf) {
    ESP_LOGE(TAG, "feed buffer alloc failed");
    vTaskDelete(nullptr);
    return;
  }

  ESP_LOGI(TAG, "feed task: chunk=%d ch=%d", chunk, nch);

  while (s_task_run) {
    if (!s_mic || !s_mic->read_raw_pcm(buf, chunk)) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    s_afe_handle->feed(s_afe_data, buf);
  }

  free(buf);
  vTaskDelete(nullptr);
}

static void sr_detect_task(void *arg) {
  (void)arg;
  int wakeup = 0;
  esp_mn_iface_t *multinet = nullptr;
  model_iface_data_t *mn_data = nullptr;

#if !CONFIG_APP_SR_WAKE_ONLY
  char *mn_name = esp_srmodel_filter(s_models, ESP_MN_PREFIX, ESP_MN_CHINESE);
  if (mn_name) {
    multinet = esp_mn_handle_from_name(mn_name);
    mn_data = multinet->create(mn_name, 6000);
    int afe_chunk = s_afe_handle->get_fetch_chunksize(s_afe_data);
    int mn_chunk = multinet->get_samp_chunksize(mn_data);
    if (afe_chunk != mn_chunk) {
      ESP_LOGW(TAG, "AFE/MN chunk mismatch: %d vs %d", afe_chunk, mn_chunk);
    }
    sr_setup_commands(multinet, mn_data);
    ESP_LOGI(TAG, "MultiNet ready: %s", mn_name);
  } else {
    ESP_LOGW(TAG, "No Chinese MultiNet model in flash");
  }
#endif

  ESP_LOGI(TAG, "detect task started, say wake word then command");

  while (s_task_run) {
    afe_fetch_result_t *res = s_afe_handle->fetch(s_afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (res->wakeup_state == WAKENET_DETECTED ||
        (res->raw_data_channels > 1 && res->wakeup_state == WAKENET_CHANNEL_VERIFIED)) {
      ESP_LOGI(TAG, "Wake word detected (model=%d word=%d)",
               res->wakenet_model_index, res->wake_word_index);
      sr_emit_event(APP_SR_EVT_WAKEWORD, 0, nullptr);
      if (multinet && mn_data) {
        multinet->clean(mn_data);
      }
      wakeup = 1;
    }

#if CONFIG_APP_SR_WAKE_ONLY
    (void)wakeup;
#else
    if (!multinet || !mn_data) {
      continue;
    }

    if (wakeup) {
      esp_mn_state_t st = multinet->detect(mn_data, res->data);
      if (st == ESP_MN_STATE_DETECTING) {
        continue;
      }
      if (st == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *r = multinet->get_results(mn_data);
        if (r && r->num > 0) {
          int cid = r->command_id[0];
          ESP_LOGI(TAG, "Command detected id=%d str=%s prob=%.2f",
                   cid, r->string, r->prob[0]);
          app_sr_event_t evt = APP_SR_EVT_TIMEOUT;
          if (cid == 1) {
            evt = APP_SR_EVT_CMD_HELP;
          } else if (cid == 2) {
            evt = APP_SR_EVT_CMD_ALARM;
          } else if (cid == 3) {
            evt = APP_SR_EVT_CMD_MESSAGE;
          }
          sr_emit_event(evt, cid, r->string);
        }
        wakeup = 0;
        s_afe_handle->enable_wakenet(s_afe_data);
        ESP_LOGI(TAG, "Listening for wake word again");
        continue;
      }
      if (st == ESP_MN_STATE_TIMEOUT) {
        esp_mn_results_t *r = multinet->get_results(mn_data);
        ESP_LOGW(TAG, "Command timeout: %s", r ? r->string : "");
        sr_emit_event(APP_SR_EVT_TIMEOUT, 0, r ? r->string : nullptr);
        wakeup = 0;
        s_afe_handle->enable_wakenet(s_afe_data);
      }
    }
#endif
  }

  if (mn_data && multinet) {
    esp_mn_commands_free();
    multinet->destroy(mn_data);
  }
  vTaskDelete(nullptr);
}

esp_err_t app_sr_start(app_sr_event_cb_t callback) {
  if (s_task_run) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!s_mic) {
    ESP_LOGE(TAG, "MEMS_MIC not set — call app_sr_bind_mic() first");
    return ESP_ERR_INVALID_STATE;
  }

  check_chip_config();

  s_models = esp_srmodel_init("model");
  if (!s_models) {
    ESP_LOGE(TAG, "esp_srmodel_init failed — check model partition");
    return ESP_FAIL;
  }

  for (int i = 0; i < s_models->num; i++) {
    ESP_LOGI(TAG, "SR model[%d]: %s", i, s_models->model_name[i]);
  }

  afe_config_t *afe_cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
  if (!afe_cfg) {
    ESP_LOGE(TAG, "afe_config_init failed");
    esp_srmodel_deinit(s_models);
    s_models = nullptr;
    return ESP_FAIL;
  }

  afe_cfg->aec_init = false;
  afe_cfg->se_init = false;
  afe_cfg->vad_init = true;
  afe_cfg->wakenet_init = true;

  s_afe_handle = esp_afe_handle_from_config(afe_cfg);
  s_afe_data = s_afe_handle->create_from_config(afe_cfg);
  afe_config_free(afe_cfg);

  if (!s_afe_data) {
    ESP_LOGE(TAG, "AFE create failed");
    esp_srmodel_deinit(s_models);
    s_models = nullptr;
    return ESP_FAIL;
  }

  if (!s_evt_queue) {
    s_evt_queue = xQueueCreate(8, sizeof(sr_queue_item_t));
  }

  s_user_cb = callback;
  s_task_run = true;
  app_mic_sr_set_feed_active(true);

  BaseType_t r0 = xTaskCreatePinnedToCore(sr_feed_task, "sr_feed", 8 * 1024, nullptr, 6,
                                          nullptr, 0);
  BaseType_t r1 = xTaskCreatePinnedToCore(sr_detect_task, "sr_detect", 8 * 1024, nullptr, 5,
                                          nullptr, 1);
  if (r0 != pdPASS || r1 != pdPASS) {
    s_task_run = false;
    app_mic_sr_set_feed_active(false);
    s_afe_handle->destroy(s_afe_data);
    s_afe_data = nullptr;
    esp_srmodel_deinit(s_models);
    s_models = nullptr;
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "ESP-SR started (wake: 你好小智)");
  return ESP_OK;
}

void app_sr_stop(void) {
  if (!s_task_run) return;
  s_task_run = false;
  vTaskDelay(pdMS_TO_TICKS(200));
  app_mic_sr_set_feed_active(false);
  if (s_afe_handle && s_afe_data) {
    s_afe_handle->destroy(s_afe_data);
    s_afe_data = nullptr;
  }
  if (s_models) {
    esp_srmodel_deinit(s_models);
    s_models = nullptr;
  }
  s_user_cb = nullptr;
  ESP_LOGI(TAG, "ESP-SR stopped");
}

bool app_sr_is_running(void) { return s_task_run; }

bool app_sr_poll_event(app_sr_event_t *evt, int *command_id) {
  if (!evt || !s_evt_queue) return false;
  sr_queue_item_t item;
  if (xQueueReceive(s_evt_queue, &item, 0) != pdTRUE) {
    return false;
  }
  *evt = item.evt;
  if (command_id) {
    *command_id = item.command_id;
  }
  return true;
}

/* Called from main after mic init */
extern "C" void app_sr_bind_mic(MEMS_MIC *mic) { s_mic = mic; }
