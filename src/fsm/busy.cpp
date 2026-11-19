/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习).
 *            The code ADAPTS the corresponding hardware circuit board(智能汽车-ICAR),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file busy.cpp
 * @author Leo (leo@saishukeji.com)
 * @brief 施工区控制
 * @version 0.1
 * @date 2026-09-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fsm/busy.hpp"

/**
 * @brief Construct a new Fsm Busy
 *
 * @param par
 */
FsmBusy::FsmBusy(std::shared_ptr<Params> par)
    : FSMState(FsmMode::BUSY, par)
{
}

/**
 * @brief Destroy the Fsm Busy
 *
 */
FsmBusy::~FsmBusy()
{
}

/**
 * @brief 检查状态切换
 *
 * @return FsmMode 切换后的状态
 */
FsmMode FsmBusy::getMode()
{
    if (step == Step::NONE || !params->config.busy)
        return FsmMode::NORMAL;
    else
        return FsmMode::BUSY;
}

/**
 * @brief 运行FSM状态（循环主程序）
 *
 */
void FsmBusy::run(Mat &img)
{
    if (!params->config.busy) // 该模式未启用
        return;

    resultObs = PredictResult();

    // 调试：步骤变化 + ENABLE 阶段的箭头情况（每 15 帧一行，防刷屏）
    {
        static int lastStep = -1;
        if ((int)step != lastStep)
        {
            printf("[Busy] step -> %d (lap=%d)\n", (int)step, params->currentLap);
            lastStep = (int)step;
        }
        static int dbgCnt = 0;
        if (step == Step::ENABLE && (++dbgCnt % 15 == 0))
        {
            int nBusy = 0, nChoice = 0;
            int bestY = 0, bestCenterY = -1;
            for (int i = 0; i < (int)params->results.size(); i++)
            {
                if (params->results[i].type == LABEL_BUSY)
                    nBusy++;
                if (params->results[i].type == LABEL_CHOICE)
                {
                    nChoice++;
                    if (params->results[i].y > bestY &&
                        params->results[i].width < 120 && params->results[i].height < 120)
                    {
                        bestY = params->results[i].y;
                        bestCenterY = bestY + params->results[i].height / 2;
                    }
                }
            }
            printf("[Busy-dbg] timeout=%d busy=%d choice=%d centerY=%d (need>%d)\n",
                   timeout, nBusy, nChoice, bestCenterY, (int)(ROWSIMAGE * 0.35));
        }
    }

    switch (step)
    {
    case Step::NONE: // AI未识别：等待施工区标志(busy)
    {
        // 只要有 busy 牌子出现在画面里就累计（不要求够近、不要求连续）：
        // 远处小牌常常只闪现一两帧，真正的"转向时机"交给 ENABLE 阶段的 choice 箭头。
        bool busyNow = false;
        for (int i = 0; i < params->results.size(); i++)
            if (params->results[i].type == LABEL_BUSY)
            {
                busyNow = true;
                break;
            }

        // 调试：每 15 帧报一次，并把画面里前 6 个检测框全打出来（看施工区附近模型认到什么）
        {
            static int nCnt = 0;
            if (++nCnt % 15 == 0)
            {
                int nBusy = 0, bestBottom = -1;
                char buf[256] = {0};
                int off = 0;
                for (int i = 0; i < (int)params->results.size() && i < 6; i++)
                {
                    const PredictResult &r = params->results[i];
                    if (r.type == LABEL_BUSY)
                    {
                        nBusy++;
                        int b = r.y + r.height;
                        if (b > bestBottom)
                            bestBottom = b;
                    }
                    off += snprintf(buf + off, sizeof(buf) - off, " %d(%.2f,%d,%d,%dx%d)",
                                    r.type, r.score, r.x, r.y, r.width, r.height);
                    if (off > (int)sizeof(buf) - 60)
                        break;
                }
                printf("[Busy-dbg] NONE busy=%d bestBottom=%d countRec=%d |%s\n",
                       nBusy, bestBottom, countRec, buf);
            }
        }

        if (busyNow)
            countRec++;
        else if (countRec > 0)
            countRec--; // 抖动容忍：漏一帧不清零
        if (countRec >= 3)
            setStep(Step::ENABLE); // 确认进入施工区
        break;
    }

    case Step::ENABLE: // 与停车场ENABLE一致，区域标志改为施工区busy
    {
        countSes++;
        if (!params->ctrl.stop)
            timeout++;
        if (timeout > 80)
        {
            reset();
            break;
        }

        if (findSymbols(params->results, LABEL_BUSY))
            countSes = 0;
        if (countSes > 30) // 与停车场相同：区域标志消失后的转向兜底
            setStep(Step::FORKIN);

        PredictResult choice;
        choice.score = 0;
        choice.y = 0;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_CHOICE)
            {
                if (params->results[i].y > choice.y &&
                    params->results[i].width < 120 && params->results[i].height < 120)
                    choice = params->results[i];
            }
        }
        if (choice.score > 0)
        {
            countSes = 0; // 看到入口箭头时仅根据位置触发，和停车场一致
            if ((choice.y + choice.height / 2) > ROWSIMAGE * 0.35)
                countRec++;
            if (countRec > 1)
                setStep(Step::FORKIN);
        }
        break;
    }

    case Step::FORKIN: // 使用停车场同款左转补线，施工区不根据gate结束转向
    {
        params->busyZone = true; // 施工区限速驻留：出区后继续保持，直到下一圈斑马线换圈
        timeout++;
        replanTracking(true);
        const bool finished = timeout > 89; // 入弯保持约3秒（30fps × 3s = 90帧）
        if (timeout == 1 || finished)
        {
            printf("[Busy-turn] lap=%d parking-style hold=%d/90 speed=%.2f end=%s\n",
                   params->currentLap, timeout, params->config.velBusy,
                   finished ? "max-hold" : "none");
            fflush(stdout);
        }
        if (finished)
            setStep(Step::RUNNING);
        break;
    }

    case Step::RUNNING: // 区内巡线
    {
        //[03] 出区检测：左转标志
        timeout++;
        PredictResult resLeft;
        resLeft.score = 0;
        for (int i = 0; i < params->results.size(); i++) // 搜索左转箭头
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120 &&
                params->results[i].score > resLeft.score)
                resLeft = params->results[i]; // 取置信度最高的左转箭头
        }
        if (resLeft.score > 0 && (resLeft.y + resLeft.height / 2) > ROWSIMAGE * 0.55)
            countRec++; // 左转标志开到画面下
        if (countRec > 2)
            setStep(Step::FORKOUT); // 出区左拐
        if (timeout > 120)          // 兜底：区内 4 秒没等到左转标志也要出区，否则会永远停在 BUSY（0.4 巡航）
        {
            timeout = 0;
            countRec = 0;
            setStep(Step::FORKOUT);
        }
        break;
    }

    case Step::FORKOUT: // 出区岔路转向（左拐）
    {
        timeout++;
        replanTracking(true); // 车道线重绘

        // 搜索左转标志：转向期间仍看到则hold，防止过早切出
        bool leftSign = false;
        for (int i = 0; i < params->results.size(); i++)
        {
            if (params->results[i].type == LABEL_LEFT &&
                params->results[i].width < 100 &&
                params->results[i].height < 120)
            {
                leftSign = true;
                break;
            }
        }
        if (leftSign)
            timeout = 0; // 左转标志未丢失，继续转向

        if (timeout > 29) // 转向超时（延长0.5s：14帧 + 15帧）
        {
            setStep(Step::NONE);       // 回到起始状态
        }
        break;
    }
    }
}

/**
 * @brief 图形化显示FSM数据
 *
 * @param img
 */
void FsmBusy::show(Mat &img)
{
    if (step == Step::NONE || params->mode != FsmMode::BUSY)
        return;

    if (resultObs.x > 0 && resultObs.y > 0)
    {
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }

    putText(img, "[1] Busy", Point(COLSIMAGE / 2 - 50, 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);
}

/**
 * @brief 设置新状态
 *
 * @param st
 */
void FsmBusy::setStep(Step st)
{
    step = st;
    countRec = 0; // AI场景识别计数器
    countSes = 0; // 场次计数器
    timeout = 0;  // 超时计数器
}

/**
 * @brief 车道线重绘（岔路转向）
 *
 * @param left true：左转 | false：右转
 */
void FsmBusy::replanTracking(bool left)
{
    params->track->pointsEdgeLeft.clear();  // 清空原来数据
    params->track->pointsEdgeRight.clear(); // 清空原来数据

    if (left)
    {
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, 1);                                                // 补线起点:固定左下角
        PointX endPoint = PointX(ROWSIMAGE / 3, 1);                                                   // 补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线：与停车场路口左转replanTracking()使用相同控制点
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.8);
        endPoint = PointX(ROWSIMAGE / 3, 30);
        midPoint = PointX((startPoint.x + endPoint.x) * 0.5,
                         (startPoint.y + endPoint.y) * 0.35);
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
    else
    {
        // 左车道线
        PointX startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE * 0.2);                                  // 补线起点:固定左下角
        PointX endPoint = PointX(ROWSIMAGE / 3, COLSIMAGE - 1);                                       // 补线终点
        PointX midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 补线中点
        vector<PointX> repairPoints = {startPoint, midPoint, endPoint};
        vector<PointX> modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeLeft = modifyEdge;
        // 右车道线
        startPoint = PointX(ROWSIMAGE - 10, COLSIMAGE - 1);                                    // 补线起点:固定左下角
        endPoint = PointX(ROWSIMAGE / 3, COLSIMAGE - 1);                                       // 补线终点
        midPoint = PointX((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) / 2); // 补线中点
        repairPoints = {startPoint, midPoint, endPoint};
        modifyEdge = Bezier(0.02, repairPoints); // 三阶贝塞尔曲线拟合
        params->track->pointsEdgeRight = modifyEdge;
    }
}

/**
 * @brief 搜索AI标志
 *
 * @param label
 * @return true
 * @return false
 */
bool FsmBusy::findSymbols(vector<PredictResult> results, int label)
{
    for (int i = 0; i < results.size(); i++)
    {
        if (results[i].type == label) // AI标志
            return true;
    }

    return false;
}

/**
 * @brief 缩减优化车道线（双车道→单车道）
 *
 * @param left
 */
void FsmBusy::curtailTracking(bool left)
{
    if (left) // 向左侧缩进
    {
        if (params->track->pointsEdgeRight.size() > params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeRight.resize(params->track->pointsEdgeLeft.size());

        for (int i = 0; i < params->track->pointsEdgeRight.size(); i++)
        {
            params->track->pointsEdgeRight[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
    else // 向右侧缩进
    {
        if (params->track->pointsEdgeRight.size() < params->track->pointsEdgeLeft.size())
            params->track->pointsEdgeLeft.resize(params->track->pointsEdgeRight.size());

        for (int i = 0; i < params->track->pointsEdgeLeft.size(); i++)
        {
            params->track->pointsEdgeLeft[i].y = (params->track->pointsEdgeRight[i].y + params->track->pointsEdgeLeft[i].y) / 2;
        }
    }
}

/**
 * @brief 复位流程状态（远程接管结束/换圈时调用）
 */
void FsmBusy::reset()
{
    setStep(Step::NONE);
    enable = false;
    slowing = false;
    slowingIn = false;
    resultObs = PredictResult();
}

void FsmBusy::resetLap()
{
    reset();
    manualTakeover = false;
    params->busyZone = false; // 换圈（识别到斑马线）后解除施工区限速
}