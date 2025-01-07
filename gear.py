import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80
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
            movej,
            movel,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            move_periodic,
            DR_BASE,
            trans,
            amove_periodic,
            DR_TOOL,
            move_periodic,
            drl_script_stop,
            DR_QSTOP,
            get_current_posx,
        )
        
    except ImportError as e:
        print(f"Error importing ESR_ROBOT2 : {e}")
        return
        
    def grip():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        wait(2)
        
    def release():
        set_digital_output(2,ON)
        set_digital_output(1,OFF)
        wait(1)

    # 기어 초기 위치 작은기어1 순서
    origin_pose = [278.380, -1.2, 330-228.6, 178.62, 180, 179.03]
        
    # 기어 목표 위치 작은기어1 순서
    goal_pose = [563.530, -1.3, 330-228.6, 100.78, -178.93, 100.09]

    # tcp 무게 설정
    set_tool("Tool Weight_2FG")
    # tcp 설정
    set_tcp("2FG_TCP")
    
    HOME = [0,0,90,0,90,0]

    while rclpy.ok():
        # 그립 열기
        release()
        
        # 홈위치로 이동
        movej(HOME, vel = VELOCITY, acc = ACC)

        # 기어 위치로 이동
        movel(origin_pose, vel = VELOCITY, acc = ACC)
        
        # 아래로 이동
        movel(trans(origin_pose, [0,0,-60,0,0,0], DR_BASE, DR_BASE), vel = VELOCITY, acc = ACC)
        
        # 그립 닫기
        grip()
        
        # 수직으로 위로 이동
        movel(origin_pose, vel = VELOCITY, acc = ACC)
        
        # 목표로 이동
        movel(goal_pose, vel = VELOCITY, acc = ACC)
        
        # 힘제어 키기
        task_compliance_ctrl()
        set_desired_force([0,0,-15,0,0,0], [0,0,1,0,0,0], mod = DR_FC_MOD_REL)
        
        # Z축 외력 기준 초과 확인
        while not check_force_condition(DR_AXIS_Z, max = 10):
            pass

        # 비틀어 넣기
        amove_periodic(amp = [0,0,0,0,0,10], period = 2, atime = 0.5, repeat = 5, ref = DR_TOOL)

       # 높이 감지 후 비틀기 종료
        while True:
            wait(1)
            if get_current_posx()[0][2] < 50:
                drl_script_stop(DR_QSTOP)
                break

        # 힘제어 끄기
        release_compliance_ctrl()
        wait(1)

        # 그립 열기
        release()
        
        # 수직으로 위로 이동
        movel(goal_pose, vel = VELOCITY, acc = ACC)

        # 홈위치로 이동
        movej(HOME, vel = VELOCITY, acc = ACC)

        break

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
