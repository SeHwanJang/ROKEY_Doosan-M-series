import rclpy
import DR_init
import numpy as np

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80
OFF, ON = 0, 1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("domino", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            wait,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_BASE,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            check_force_condition,
            DR_AXIS_Z,
            release_compliance_ctrl,
        )

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 :{e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2)

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(2)


    def quadratic_bezier(p0, p1, p2, t):
        return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2
    

    # 베지어 곡선 첫번째 점
    Bezier_point1 = np.array([640, 180])
    # 베지어 곡선 두번째 점
    Bezier_point2 = np.array([460, -180])    
    # 베지어 곡선 세번째 점
    Bezier_point3 = np.array([280, 180])
    # 목표 도미노 개수
    goal_domino_num = 20

    # 도미노 놓을 포인트
    t_values = np.linspace(0,1,goal_domino_num)
    curve_points = np.array([quadratic_bezier(Bezier_point1, Bezier_point2, Bezier_point3, t) for t in t_values])
    print(curve_points)


    # tcp 무게 설정
    set_tool("Tool Weight_2FG")
    # tcp 설정
    set_tcp("2FG_TCP")


    # 도미노 받는 위치(joint 기준)
    HOME = [0,0,120,0,0,0]

    # 놓은 도미노 숫자
    now_domino_num = 0
    
    # 첫 도미노 놓을 위치(pose 기준)
    x = curve_points[now_domino_num,0]
    y = curve_points[now_domino_num,1]
    Goal_pose = [x, y, 150, 160, -180, -20]


    while rclpy.ok():
        # 도미노 줄 위치로 이동
        movej(HOME, vel = VELOCITY, acc = ACC)

        # 물체 잡을 때까지 반복 
        while True:
            # 그립 열기
            release()

            # 그립 닫기
            grip()

            if get_digital_input(1) == ON and get_digital_input(2) == OFF:
                break
        
        # 특이점 방지 자세 보정
        movej([0,0,90,0,90,0], vel = VELOCITY, acc = ACC)

        # 도미노 놓을 위치로 이동
        movel(Goal_pose, vel = VELOCITY, acc = ACC)
        
        # 힘제어 키기
        task_compliance_ctrl()
        set_desired_force([0,0,-20,0,0,0], [0,0,1,0,0,0], mod = DR_FC_MOD_REL)

        # 바닥에 닿을 때까지 아래로
        while not check_force_condition(DR_AXIS_Z, max = 5):
            pass
        
        # 힘제어 끄기
        release_compliance_ctrl()
        wait(1)

        # 그립 열기
        release()

        # 놓은 도미노 수 증가
        now_domino_num += 1

        # 수직으로 위로 이동
        movel(Goal_pose, vel = VELOCITY, acc = ACC)

        # 목표 도미노 수와 같으면 종료
        if now_domino_num == goal_domino_num:
            break


        # 다음에 도미노 놓을 위치 수정
        x = curve_points[now_domino_num, 0]
        y = curve_points[now_domino_num, 1]

        # 그랩 방향 조절
        grip_way = (90 // (goal_domino_num // 2))

        # 다음 도미노 좌표
        Goal_pose = [x, y, 150, 160, -180, -20 + grip_way*now_domino_num]

        # 다음 도미노 좌표 출력
        print(Goal_pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()



# import matplotlib.pyplot as plt
# import numpy as np

# def quadratic_bezier(p0, p1, p2, t):
#     return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2

# # Control points
# p0 = np.array([0, 0])
# p1 = np.array([1, 2])
# p2 = np.array([2, 0])

# # Generate the points of the Bezier curve
# t_values = np.linspace(0, 1, 100)
# curve_points = np.array([quadratic_bezier(p0, p1, p2, t) for t in t_values])

# # Plotting the curve
# plt.plot(curve_points[:, 0], curve_points[:, 1], label="Quadratic Bezier Curve")
# plt.scatter([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]], color='red')  # Control points
# plt.plot([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]], 'r--', label="Control Polygon")
# plt.legend()
# plt.title("Quadratic Bezier Curve")
# plt.xlabel("x")
# plt.ylabel("y")
# plt.grid(True)
# plt.axis("equal")
# plt.show()
