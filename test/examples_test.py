import pytest
import os


def test_case1():
    os.system("python ./examples/particle_motion.py")


def test_case2():
    os.system("python ./examples/positional_pid_control.py")


def test_case3():
    os.system("python ./examples/incremental_pid_control.py")


def test_case4():
    os.system("python ./examples/pure_pursuit_control.py")


def test_case5():
    os.system("python ./examples/stanley_control.py")


def test_case6():
    os.system("python ./examples/lqr_control.py")


if __name__ == "__main__":
    pytest.main()
