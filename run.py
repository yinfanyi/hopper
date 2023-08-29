from mujoco_base import MuJoCoBase


def main():
    
    # from examples.biped import Biped
    # xml_path = "./xml/biped.xml"
    # sim = Biped(xml_path)
    
    # from examples.hopper import Hopper
    # xml_path = "./xml/hopper.xml"
    # sim = Hopper(xml_path)
    
    # from examples.a1_env import A1Sim
    # xml_path = "./xml/a1/xml/a1.xml"
    # sim = A1Sim(xml_path)
    
    # from examples.control_pendulum import ContolPendulum
    # xml_path = "./xml/pendulum.xml"
    # sim = ContolPendulum(xml_path)
    
    # from examples.dbpendulum import DoublePendulum
    # xml_path = "./xml/doublependulum.xml"
    # sim = DoublePendulum(xml_path)
    
    # from examples.gymnast import Gymnast
    # xml_path = "./xml/gymnast.xml"
    # sim = Gymnast(xml_path)
    
    # from examples.iiwa import Iiwa
    # xml_path = "./xml/iiwa/iiwa.xml"
    # sim = Iiwa(xml_path)
    
    # from examples.ivp import InitialValueProblem
    # xml_path = "./xml/projectile_opt.xml"
    # sim = InitialValueProblem(xml_path)
    
    # from examples.jaco2 import Jaco2
    # xml_path = "./xml/jaco2/jaco2.xml"
    # sim = Jaco2(xml_path)
    
    # from examples.leg_swing import LegSwing
    # xml_path = "./xml/doublependulum_leg.xml"
    # sim = LegSwing(xml_path)
    
    # from examples.manipulator_drawing import ManipulatorDrawing
    # xml_path = "./xml/doublependulum_manipulator.xml"
    # sim = ManipulatorDrawing(xml_path)
    
    # from examples.manipulator_ik import ManipulatorIK
    # xml_path = "./xml/manipulator.xml"
    # sim = ManipulatorIK(xml_path)
    # sim.reset()
    # sim.simulate()
    # sim.visualize()
    
    from examples.underactuated_pendulum import Acrobot
    xml_path = "./xml/acrobot.xml"
    sim = Acrobot(xml_path)
    
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
