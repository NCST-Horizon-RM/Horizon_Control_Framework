#include "Contact_Detector.h"
#include "contact_detector_model.h"
#include <math.h>
#include <stddef.h>

/** 限制 sigmoid 输入，避免指数函数溢出。 */
#define CONTACT_DETECT_SCORE_LIMIT 20.0f

bool Contact_Detector_Update(Contact_Detector_t *detector,
                             const VMC_LegSite_t *leg,
                             float accel_z_mps2)
{
    if (detector == NULL) {
        return false;
    }
    if (leg == NULL) {
        detector->confirm_count = 0U;
        return detector->is_contact;
    }

    /* 顺序须与训练脚本 FEATURE_NAMES 及生成模型表严格一致。 */
    const float theta = leg->theta[0];
    if (!isfinite(theta)) {
        detector->confirm_count = 0U;
        return detector->is_contact;
    }
    const float features[CONTACT_FEATURE_COUNT] = {
        leg->support_force, leg->support_torque,
        theta, leg->theta[1], leg->theta[1] * leg->theta[1],
        leg->theta[2], sinf(theta), cosf(theta),
        leg->length[0], leg->length[1], leg->length[2], accel_z_mps2
    };

    /* 先以训练集均值/标准差归一化，再计算逻辑回归得分。 */
    float score = contact_model_weights[0];
    for (uint32_t index = 0U; index < CONTACT_FEATURE_COUNT; ++index) {
        if (!isfinite(features[index]) || !isfinite(contact_feature_scale[index]) ||
            contact_feature_scale[index] <= 0.0f) {
            detector->confirm_count = 0U;
            return detector->is_contact;
        }
        score += contact_model_weights[index + 1U] *
                 ((features[index] - contact_feature_mean[index]) /
                  contact_feature_scale[index]);
    }
    if (!isfinite(score)) {
        detector->confirm_count = 0U;
        return detector->is_contact;
    }
    score = fmaxf(-CONTACT_DETECT_SCORE_LIMIT,
                  fminf(score, CONTACT_DETECT_SCORE_LIMIT));
    detector->contact_probability = 1.0f / (1.0f + expf(-score));

    /* 接地和离地分别使用不同阈值；中间区域保持状态并重新计数。 */
    const bool switch_requested = detector->is_contact
        ? (detector->contact_probability <= CONTACT_DETECT_EXIT_THRESHOLD)
        : (detector->contact_probability >= CONTACT_DETECT_ENTER_THRESHOLD);
    if (!switch_requested) {
        detector->confirm_count = 0U;
    } else {
        ++detector->confirm_count;
        if (detector->confirm_count >= CONTACT_DETECT_CONFIRM_SAMPLES) {
            detector->is_contact = !detector->is_contact;
            detector->confirm_count = 0U;
        }
    }
    return detector->is_contact;
}

void Contact_Detector_Init(Contact_Detector_t *detector, bool initial_contact)
{
    if (detector == NULL) {
        return;
    }
    detector->contact_probability = initial_contact ? 1.0f : 0.0f;
    detector->confirm_count = 0U;
    detector->is_contact = initial_contact;
}
