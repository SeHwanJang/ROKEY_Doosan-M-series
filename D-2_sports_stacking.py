import rclpy
import DR_init
import math

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200
OFF, ON = 0, 1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("gear", namespace=ROBOT_ID)
    
    DR_init.__dsr__node = node
    
    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            wait,
            set_tool,
            set_tcp,
            movel,
            movej,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            trans,
        )
        
    except ImportError as e:
        print(f"Error importing ESR_ROBOT2 : {e}")
        return
        
    def grip():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        wait(1)
        
    def release():
        set_digital_output(2,ON)
        set_digital_output(1,OFF)
        wait(1)

    # 목표 위치
    target = [600, -120, 190, 154, 180, 175]

    # 마지막 컵 잡는 joint
    filp_robot = [49.19, 51.83, 90.23, -45.89, 118.50, 63.81]

    # tcp 무게 설정
    set_tool("Tool Weight_2FG")
    # tcp 설정
    set_tcp("2FG_TCP")
    
    # 홈위치
    HOME = [340, 100, 195.4, 127.05, 179.96, 127.1]

    # 총 컵 개수
    total_cup_num = 11

    # 컵 지름
    size = 80

    # 정삼각형 높이
    height = size * math.sin(math.pi * (60/180))

    while rclpy.ok():
        for cup_num in range(0, total_cup_num):
            # 목표 좌표 출력
            print(target)

            # 홈위치로 이동
            movel(trans(HOME, [0,0,50,0,0,0], DR_BASE, DR_BASE), vel = VELOCITY, acc = ACC)

            # 그립 열기
            release()

            # 컵 뒤집기
            if cup_num == 10:

                # 컵 옆으로 이동
                movej(filp_robot, vel = 10, acc = 10)
                movel([-3,0,-10,0,0,0], vel = 10, acc = 10, mod = DR_FC_MOD_REL)
                
                # 그립 닫기
                grip()

                # 수직으로 위로 이동
                movel([0,0,210,0,0,0], vel = VELOCITY, acc = ACC, mod = DR_FC_MOD_REL)

                # 컵 뒤집기
                movej([0,0,0,0,0, -180], vel = 100, acc = 100, mod = DR_FC_MOD_REL)

                # 쌓은 것 옆으로 이동
                movel([target[0]-HOME[0],-(HOME[1]-target[1]),0,0,0,0], vel = 100, acc = 100, mod = DR_FC_MOD_REL)

                # 힘제어 키기
                task_compliance_ctrl()
                set_desired_force([0,0,-20,0,0,0], [0,0,1,0,0,0], mod = DR_FC_MOD_REL)
                
                # 바닥에 닿을 때까지 아래로
                while not check_force_condition(DR_AXIS_Z, max = 8):
                    pass
                    
                # 힘제어 끄기
                release_compliance_ctrl()
                wait(1)

                # 그립 열기
                release()

                # 옆으로 빠지기
                movel([0,70,0,0,0,0], vel = 100, acc = 100, mod = DR_FC_MOD_REL)
                movel([-100,0,0,0,0,0], vel = 100, acc = 100, mod = DR_FC_MOD_REL)

                # 홈으로 이동
                movej([0,0,90,0,90,0], vel = 30, acc = 30)

                return

            # 아래로 이동
            movel(trans(HOME, [0,0,-3-11*cup_num,0,0,0], DR_BASE, DR_BASE), vel = VELOCITY, acc = ACC)

            # 그립 닫기
            grip()

            # 수직으로 위로 이동
            movel(trans(HOME, [0,0,130,0,0,0], DR_BASE, DR_BASE), vel = VELOCITY, acc = ACC)

            # 목표 좌표 이동
            movel(target, vel = VELOCITY, acc = ACC)

            # 힘제어 키기
            task_compliance_ctrl()
            set_desired_force([0,0,-40,0,0,0], [0,0,1,0,0,0], mod = DR_FC_MOD_REL)
            
            # 바닥에 닿을 때까지 아래로
            while not check_force_condition(DR_AXIS_Z, max = 8):
                pass
                
            # 힘제어 끄기
            release_compliance_ctrl()
            wait(1)
            
            # 그립 열기
            release()

            # 수직으로 위로 이동
            movel(trans(target, [0,0,25,0,0,0], DR_BASE, DR_BASE), vel = VELOCITY, acc = ACC)


            # 1층 2개
            if cup_num == 2:
                target[0] -= height
                target[1] -= (size//2 + size)
            # 1층 1개
            elif cup_num == 4:
                target[0] -= height
                target[1] -= (size//2)
            # 2층 이동
            elif cup_num == 5:
                target[0] += (height + height*(2/3))
                target[1] -= (size//2)
                target[2] += 30
            # 2층 2개
            elif cup_num == 7:
                target[0] -= height
                target[1] -= (size//2)
            # 2층 1개
            elif cup_num == 8:
                target[0] += (height*(2/3))
                target[2] += 70
            # 3층 1개
            elif cup_num == 9:
                pass
            # 기본 좌표 수정
            else:
                target[1] += size


    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
