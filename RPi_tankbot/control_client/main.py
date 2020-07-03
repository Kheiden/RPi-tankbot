import requests
import pygame
#TODO: Get protos working on windows
#import protos.controllers_pb2

class ControlWindow():
  def __init__(self):
    """Run when a new Control Window is instantiated."""
    #self.all_controllers = controllers_pb2.Controllers()

  def start_control_window(self):
    """Run when a new Control Window is instantiated."""


    # Define some colors.
    BLACK = pygame.Color('black')
    WHITE = pygame.Color('white')

    IP_ADDRESS = '192.168.1.16'
    PORT = '5000'

    # This is a simple class that will help us print to the screen.
    # It has nothing to do with the joysticks, just outputting the
    # information.
    class TextPrint(object):
        def __init__(self):
            self.reset()
            self.font = pygame.font.Font(None, 20)

        def tprint(self, screen, textString):
            textBitmap = self.font.render(textString, True, BLACK)
            screen.blit(textBitmap, (self.x, self.y))
            self.y += self.line_height

        def reset(self):
            self.x = 10
            self.y = 10
            self.line_height = 15

        def indent(self):
            self.x += 10

        def unindent(self):
            self.x -= 10

    class Callback (object):
      def __init__(self):
        self.connection_failure_count = 0
        self.connection_200_count = 0
        self.request_previous = {
          'joystick_name': 'DEFAULT joystick_name',
          'axis_name': 'DEFAULT axis_name',
          'axis_value': -1.321,
        }

      def tread_controller(self, axis_name, axis_value):
        """
        This function sends outbound REST API calls to control tread tracks.

        Args:
          axis_name: One of 'Axis 0' (left tread) or 'Axis 1' (right tread)
          axis_value: 1 is full reverse, -1 is full forward, 0 is full stop
        """
        #print("Axis Updated to new value:", axis_value)
        payload = {'axis_name': axis_name, 'axis_value': axis_value}
        endpoint = 'v2/move'
        try:
          r = requests.post('http://{}:{}/{}'.format(IP_ADDRESS, PORT, endpoint),
            timeout=0.001)
          if r.status_code == 200:
            self.connection_200_count += 1
          print(r.status_code)
        except:
          self.connection_failure_count += 1
          print("Current Connection Failure Count:", self.connection_failure_count)

      def axis_update(self, joystick_name, axis_name, axis_value):
        """
        This function is called each time there is an update to the value of
        any axis as recorded by the main thread.

        Args:
          joystick_name: The name of the joystick which has an update
          axis_name: The name of the axis which has an update
          axis_value: The new value of the axis
        """

        list_of_axis = ["Axis 0", "Axis 1"]
        if axis_name in list_of_axis:
          self.request_previous['joystick_name'] = joystick_name
          self.request_previous['axis_name'] = axis_name
          self.request_previous['axis_value'] = axis_value
          self.tread_controller(axis_name=axis_name, axis_value=axis_value)

      def button_update(self, joystick_name, button_number, button_value):
        """
        Args:
          joystick_name: The name of the joystick which has an update
          button_number: The number of the button which has an update
          button_value: The new value of the button
        """
        pass

    pygame.init()

    # Set the width and height of the screen (width, height).
    screen = pygame.display.set_mode((500, 700))

    pygame.display.set_caption("My Game")

    # Loop until the user clicks the close button.
    done = False

    # Used to manage how fast the screen updates.
    clock = pygame.time.Clock()

    # Initialize the joysticks.
    pygame.joystick.init()

    # Get ready to print data to screen
    textPrint = TextPrint()
    callback = Callback()

    # -------- Main Program Loop -----------
    while not done:
        #
        # EVENT PROCESSING STEP
        #
        # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # If user clicked close.
                done = True # Flag that we are done so we exit this loop.
            elif event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

        #
        # DRAWING STEP
        #
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
        screen.fill(WHITE)
        textPrint.reset()

        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()
        #self.all_controllers.count_of_joysticks = pygame.joystick.get_count()

        textPrint.tprint(screen, "Number of joysticks: {}".format(joystick_count))
        textPrint.indent()

        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            #joystick_proto = self.all_controllers.joysticks.add()
            #joystick_proto.index_location = i


            textPrint.tprint(screen, "Joystick {}".format(i))
            textPrint.indent()

            # Get the name from the OS for the controller/joystick.
            joystick_name = joystick.get_name()
            #joystick_proto.name = name
            textPrint.tprint(screen, "Joystick name: {}".format(joystick_name))

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            #joystick_proto.number_of_axes = axes
            textPrint.tprint(screen, "Number of axes: {}".format(axes))
            textPrint.indent()

            for i in range(axes):
                axis_value = joystick.get_axis(i)
                #new_axis = joystick_proto.axis.add()
                #new_axis.axis_index_location = i
                axis_name = 'Axis {}'.format(i)
                #new_axis.axis_name = axis_name
                #new_axis.axis.value = axis
                callback.axis_update(joystick_name, axis_name, axis_value)
                textPrint.tprint(screen, "{} value: {:>6.3f}".format(axis_name, axis_value))
            textPrint.unindent()

            buttons = joystick.get_numbuttons()
            #joystick_proto.count_of_buttons = buttons
            textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
            textPrint.indent()

            for i in range(buttons):
              #new_button = joystick_proto.button.add()
              #new_button.index_location = i
              button = joystick.get_button(i)
              #new_button.toggled = button
              callback.button_update(joystick_name=joystick_name,
                button_number=i,
                button_value=button)
              textPrint.tprint(screen,
                                 "Button {:>2} value: {}".format(i, button))
            textPrint.unindent()

            hats = joystick.get_numhats()
            textPrint.tprint(screen, "Number of hats: {}".format(hats))
            textPrint.indent()

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
            textPrint.unindent()

            textPrint.unindent()

        #
        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
        #

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # Limit to 20 frames per second.
        clock.tick(20)

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()

  def get_current_joystick_position(self, joystick_name):
    """
    Args:
      joystick_name: This is the name of the joystick

    Joystick name: Saitek Pro Flight X-56 Rhino
    Axis 0 is left
    Axis 1 is right

    """
    pass



class RhinoThrottle():
  pass

class Joystick():
  pass

if __name__ == '__main__':
  control_window = ControlWindow()
  control_window.start_control_window()
