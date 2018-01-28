#!/usr/bin/python

import sys
import time
import math
import numpy as np
import torch

sys.path.append('../lib/python/arm64')
import robot_interface as sdk


def jointLinearInterpolation(initPos, targetPos, rate):

    rate = np.fmin(np.fmax(rate, 0.0), 1.0)
    p = initPos*(1-rate) + targetPos*rate
    return p

def mcu_comm_init(ctx: dict) -> None:
    LOWLEVEL  = 0xff
    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    cmd = sdk.LowCmd()
    udp.InitCmdData(cmd)
    safe = sdk.Safety(sdk.LeggedType.Go1)
    state = sdk.LowState()
    ctx['cmd'] = cmd
    ctx['udp'] = udp
    ctx['safe'] = safe
    ctx['state'] = state


def recv_proprioception(ctx: dict, dtype: torch.dtype) -> torch.Tensor:
    '''
    mortor_idex = [
    'FL_0', 'FL_1', 'FL_2', 
    'FR_0', 'FR_1', 'FR_2', 
    'RL_0', 'RL_1', 'RL_2', 
    'RR_0', 'RR_1', 'RR_2',
    ]
    '''
    udp = ctx['udp']  # unitree sdk.udp
    state = ctx['state']
    udp.Recv()
    udp.GetRecv(state)
    position = torch.Tensor([state.motorState[i].q for i in range(12)])
    velocity = torch.Tensor([state.motorState[i].dq for i in range(12)])
    Euler_angle = torch.Tensor([state.imu.rpy[i] for i in range(3)])  # Roll Pitch Yaw
    foot_contact = torch.Tensor([state.footForce[i] for i in range(4)])  # values of four foot sensors
    input_state = torch.cat([position, velocity, Euler_angle, foot_contact]).unsqueeze(0)  # feeded to learned policy

    return input_state


def send_action(ctx: dict, action: torch.Tensor):
    cmd = ctx['cmd']
    udp = ctx['udp']
    safe = ctx['safe']
    state = ctx['state']
    Kp = [40] * 12
    Kd = [0.5] * 12
    action = action.squeeze(0)
    # action ot command
    for i in range(12):
        cmd.motorCmd[i].q = action[i]  # expected position
        cmd.motorCmd[i].dq = 0         # expected velocity
        cmd.motorCmd[i].Kp = Kp[i]     # position stiffness
        cmd.motorCmd[i].Kd = Kd[i]     # velocity stiffness
        cmd.motorCmd[i].tau = 0.0      # expected torque

    # safety check
    safe.PowerProtect(cmd, state, 1)
    # send and execute action
    udp.SetSend(cmd)
    udp.Send()




if __name__ == '__main__':
    d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
         'FL_0':3, 'FL_1':4, 'FL_2':5, 
         'RR_0':6, 'RR_1':7, 'RR_2':8, 
         'RL_0':9, 'RL_1':10, 'RL_2':11 }
    ctx={}
    mcu_comm_init(ctx=ctx)
    Tpi = 0
    motiontime = 0
    PosStopF  = math.pow(10,9)
    VelStopF  = 16000.0
    init_qpos = np.array([0.] * 12)
    ready_qpos = np.array([
        0.0, 1.28, -2.7,
        0.0, 1.28, -2.7,
        0.0, 1.28, -2.7,
        0.0, 1.28, -2.7,
    ])
    desired_qpos = np.array([
        -0.1, 0.8, -1.5,
        0.1, 0.8, -1.5,
        0.1, 1.0, -1.5,
        0.1, 1.0, -1.5,
    ])
    action = np.array([0.] * 12)
    while True:
        time.sleep(0.02)
        motiontime += 1
        print(motiontime)
        input_state = recv_proprioception(ctx=ctx,dtype=torch.dtype)
        # print('position:',input_state[0][:12]) # print position

        # state = ctx['state']
        # print([state.motorState[j].q for j in range(12)])

        if( motiontime >= 0):

            # first, get record initial position
            if( motiontime >= 0 and motiontime < 100):
                state = ctx['state']
                for j in range(12):
                    init_qpos[j] = state.motorState[j].q
                    action[j] = init_qpos[j]
            
            elif motiontime < 200:
                action = jointLinearInterpolation(init_qpos, ready_qpos, (motiontime - 100.0) / 100.0)

            # stand
            elif motiontime < 400:
                action = jointLinearInterpolation(ready_qpos, desired_qpos, (motiontime - 200.0) / 200.0)
            
            # keep standing pose
            else:
                for j in range(12):
                    action[j] = desired_qpos[j]


        action_send = torch.from_numpy(action).unsqueeze(0)
        send_action(ctx=ctx,action=action_send)
