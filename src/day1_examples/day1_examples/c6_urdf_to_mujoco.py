"""6교시 실습 1: URDF → MuJoCo 모델(MJCF) 변환 과정 보기 (ROS 노드 아님, 파이썬 스크립트)

  $MUJOCO_PYTHON -m day1_examples.c6_urdf_to_mujoco            # 변환 결과 출력 + MJCF 저장
  $MUJOCO_PYTHON -m day1_examples.c6_urdf_to_mujoco --view     # MuJoCo 뷰어로 보기 (중력에 팔이 떨어짐)

보여 주는 것
  - URDF 의 link → MuJoCo body, joint → joint, <mimic> → equality(등식 구속)
  - URDF 의 토크 한계(effort) → MuJoCo 관절 토크 한계(actuatorfrcrange)
  - 변환된 MJCF 파일: 현재 폴더의 edu_arm_generated.xml
  ($MUJOCO_PYTHON 은 scripts/activate.sh 가 설정하는 MuJoCo 가상환경 파이썬)
"""
import os
import sys

import mujoco
import numpy as np

from day1_examples.c6_mujoco_bridge import ARM_JOINTS, FINGER_JOINT, build_model, load_urdf

OUT = os.path.join(os.getcwd(), 'edu_arm_generated.xml')


def main():
    urdf = load_urdf()
    raw = mujoco.MjSpec.from_string(urdf).compile()      # 액추에이터를 붙이기 전, URDF 그대로의 모델
    print('== URDF 를 그대로 MuJoCo 로 불러온 결과 ==')
    print(f'  body {raw.nbody - 1}개 (worldbody 제외), joint {raw.njnt}개, '
          f'액추에이터 {raw.nu}개, 등식 구속 {raw.neq}개')
    print('  → 액추에이터가 0개: URDF 에는 "모터" 정보가 없으므로 직접 붙여야 한다')
    print('  → 등식 구속 1개: right_finger 의 <mimic> 이 "왼쪽 손가락과 같은 값" 구속으로 바뀜\n')

    print('== 관절별 정보 (URDF <limit> 이 그대로 넘어옴) ==')
    for i in range(raw.njnt):
        j = raw.joint(i)
        lo, hi = raw.jnt_range[i]
        f = raw.jnt_actfrcrange[i][1]
        unit = 'm' if raw.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE else '°'
        scale = 1.0 if unit == 'm' else np.degrees(1.0)
        print(f'  {j.name:20s} 범위 {lo * scale:8.3f} ~ {hi * scale:7.3f} {unit}   토크 · 힘 한계 {f:6.1f}')

    initial = dict(zip(ARM_JOINTS, [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]))
    initial[FINGER_JOINT] = 0.0
    model, data, spec = build_model(urdf, initial, return_spec=True)
    print(f'\n== 바닥 · 조명 · 위치 제어 액추에이터를 붙인 최종 모델: 액추에이터 {model.nu}개 ==')
    with open(OUT, 'w') as f:
        f.write(spec.to_xml())
    print(f'  변환된 MJCF 저장: {OUT}')

    if '--view' in sys.argv:
        from mujoco import viewer as mj_viewer
        print('\n뷰어: 액추에이터 목표를 0 으로 둬서 팔이 영점 자세(수평으로 뻗음)로 움직인다. 창을 닫으면 종료')
        data.ctrl[:] = 0.0
        mj_viewer.launch(model, data)


if __name__ == '__main__':
    main()
