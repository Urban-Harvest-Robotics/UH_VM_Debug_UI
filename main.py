import rclpy
import pygame
from ros_interface import ROSInterface
from ui import AgitatorControlUI
from ui import DistanceDisplayUI
from ui import LinearGuideControlUI
from ui import FruitSolidDispenserUI  
from ui import BlenderHeaterUI
from ui import DoorControlUI
from ui import LiquidSystemUI
from ui import DCPumpUI
from ui import ConveyorSliderUI
from ui import CupBowlUI 
from ui import RecordPlayUI
from ui import BlenderPoseUI
from ui import LinearGuidePointsUI
from ui import PlayCommandUI
def main():
    rclpy.init()
    ros_node = ROSInterface()

    pygame.init()
    screen = pygame.display.set_mode((1900, 1900))
    pygame.display.set_caption("Vending Machine Debug UI")

    # Initialize UI modules
    agitator_ui = AgitatorControlUI(screen, ros_node)
    distance_ui = DistanceDisplayUI(screen, ros_node, start_y=150)
    linear_ui = LinearGuideControlUI(screen, ros_node, start_y=800)
    fruit_solid_ui = FruitSolidDispenserUI(screen, ros_node, x=1200, y=50)
    blender_ui = BlenderHeaterUI(screen, ros_node, x=600, y=100)
    door_ui = DoorControlUI(screen, ros_node, x=600, y=150)
    liquid_ui = LiquidSystemUI(screen, ros_node, x=600, y=200)
    dc_pump_ui = DCPumpUI(screen, ros_node, x=600, y=250)
    conveyor_slider_ui = ConveyorSliderUI(screen,ros_node,x=600,y=300)
    cup_bowl_ui = CupBowlUI(screen, ros_node,x=600,y=350)
    record_play_ui = RecordPlayUI(screen, ros_node, x=1200, y=800)
    blender_pose_ui = BlenderPoseUI(screen, ros_node , x=600, y=500)
    linear = LinearGuidePointsUI(screen, ros_node,x=50 , y=900 )
    play_command_ui = PlayCommandUI(screen,ros_node,x=1200,y=700)
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            agitator_ui.handle_event(event)
            linear_ui.handle_event(event)
            fruit_solid_ui.handle_event(event)  
            blender_ui.handle_event(event)
            door_ui.handle_event(event)
            liquid_ui.handle_event(event)
            dc_pump_ui.handle_event(event)
            conveyor_slider_ui.handle_event(event)
            cup_bowl_ui.handle_event(event)
            record_play_ui.handle_event(event)
            blender_pose_ui.handle_event(event)
            linear.handle_event(event)
            play_command_ui.handle_event(event)
        screen.fill((255, 255, 255))

        agitator_ui.draw()
        distance_ui.draw()
        linear_ui.draw()
        fruit_solid_ui.draw(screen)  
        blender_ui.draw(screen)
        door_ui.draw(screen)
        liquid_ui.draw(screen)
        dc_pump_ui.draw(screen)
        conveyor_slider_ui.draw()
        cup_bowl_ui.draw(screen)
        record_play_ui.draw(screen)
        blender_pose_ui.draw()
        linear.draw()
        play_command_ui.draw()
        pygame.display.flip()
        clock.tick(30)

        rclpy.spin_once(ros_node, timeout_sec=0.01)

    ros_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
