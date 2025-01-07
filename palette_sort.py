# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            get_current_posx,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            set_digital_output,
            get_digital_input,
            wait,
            drl_script_stop,
            DR_SSTOP,
            trans,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        #wait_digital_input(2)
        wait(2)

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        #wait_digital_input(1)
        wait(2)

    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 초기 팔레트 위치
    origin_pose = [[500.508, -5.833, 80, 37.006, -178.798, 36.906],
    [448.594, -5.839, 80, 35.852, -178.704, 35.724],
    [397.260, -5.842, 80, 46.926, -179.158, 47.136],
    [500.816, 44.956, 80, 31.296, -178.277, 30.618],
    [449.209, 45.378, 80, 32.405, -178.353, 31.841],
    [398.833, 46.553, 80, 26.625, -177.649, 27.251],
    [501.457, 97.877, 80, 32.331, -177.428, 30.377],
    [450.182, 96.755, 80, 29.433, -176.804, 28.869],
    [399.173, 96.792, 80, 28.432, -176.415, 27.796]]
    # 목표 팔레트 위치
    goal_pose = [[650.424, -6.413, 110, 33.611, -177.221, 33.589],
    [599.151, -5.919, 110, 31.720, -177.103, 31.565],
    [547.921, -4.874, 110, 29.908, -176.937, 30.208],
    [651.359, 44.558, 110, 34.421, -177.013, 34.662],
    [599.911, 45.546, 110, 34.953, -176.921, 34.679],
    [548.860, 46.181, 110, 27.874, -176.012, 27.496],
    [651.689, 95.342, 110, 34.874, -176.572, 34.595],
    [600.451, 96.044, 110, 34.419, -176.436, 34.000],
    [549.595, 97.298, 110, 34.162, -176.278, 33.575]]
    # 목표 팔레트 물체 존재 여부, 0없음/1있음
    goal_pose_check = [0,0,0,0,0,0,0,0,0]
    # 현재 잡은 물체 높이 순서 (낮음)0,1,2(높음)
    height = -1
    # 물체 존재 여부 보정값
    temp = 0
    

    while rclpy.ok():
        # 홈 초기화
        movej(JReady, vel = 60 , acc = 60)
        
        # 물체 sort
        for index in range(0,9):
            # 물체 위로 이동
            movel(origin_pose[index], vel = 60, acc = 60)
            # 그립 닫기
            grip()
            # compliance 제어 키기
            task_compliance_ctrl()
            # force 제어 키기, 아래로 이동
            set_desired_force(fd=[0,0,-10,0,0,0],dir=[0,0,1,0,0,0], mod = DR_FC_MOD_REL)
            # 물체에 닿을 때까지
            while not check_force_condition(DR_AXIS_Z, max = 5):
                pass
            
            # compliance 제어 끄기
            release_compliance_ctrl()
            wait(1)

            # 물체 높이 설정
            if get_current_posx(ref=DR_BASE)[0][2] >= 55:
                height = 2
            elif get_current_posx(ref=DR_BASE)[0][2] >= 45: 
                height = 1
            else:
                height = 0
            # 위로 살짝 이동
            movel(trans(origin_pose[index],[0,0,1,0,0,0],DR_BASE, DR_BASE), vel = 20, acc = 20)
            # 그립 열기
            release()
            # 아래로 이동
            movel(trans(origin_pose[index],[0,0,-45,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
            # 그립 닫기
            grip()
            # 위로 이동
            movel(trans(origin_pose[index],[0,0,50,0,0,0], DR_BASE, DR_BASE), vel = 60, acc = 60)
            # 크기 파악 후 정렬 위치로 이동, 목표 위치 물체 확인 후 보정, 물체 존재 여부 최신화
            while True:
                if height == 2 and goal_pose_check[0+temp] == 0:
                    movel(goal_pose[0+temp], vel = 60, acc = 60)
                    goal_pose_check[0+temp] = 1
                    # 아래로 이동
                    movel(trans(goal_pose[0+temp],[0,0,-60,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    # 그립 놓기
                    release()
                    # 위로 이동
                    movel(trans(goal_pose[0+temp],[0,0,10,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    break
                elif height == 1 and goal_pose_check[3+temp] == 0:
                    movel(goal_pose[3+temp], vel = 60, acc = 60)
                    goal_pose_check[3+temp] = 1
                    # 아래로 이동
                    movel(trans(goal_pose[3+temp],[0,0,-62,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    # 그립 놓기
                    release()
                    # 위로 이동
                    movel(trans(goal_pose[3+temp],[0,0,10,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    break
                elif height == 0 and goal_pose_check[6+temp] == 0:
                    movel(goal_pose[6+temp], vel = 60, acc = 60)
                    goal_pose_check[6+temp] = 1
                    # 아래로 이동
                    movel(trans(goal_pose[6+temp],[0,0,-65,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    # 그립 놓기
                    release()
                    # 위로 이동
                    movel(trans(goal_pose[6+temp],[0,0,10,0,0,0], DR_BASE, DR_BASE), vel = 20, acc = 20)
                    break
                temp += 1
            # 잡은 물체 높이 초기화
            height = -1
            # 물체 존재여부 보정값 초기화
            temp = 0

        break
        
    rclpy.shutdown()



if __name__ == "__main__":
    main()
