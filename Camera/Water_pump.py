class Water_pump:

    CHIP_NAME      = 'gpiochip4'
    LINE_NUMBERS   = [21,20,26]

    def __init__(self, pins: list=[21,20,26], debug: bool=False) -> None:
        """
            Params: 
                pins:   
                    List of pins that are going to be 
                    outputting 1, for the relay module in the pi
                    use pin 21, 20, and 26 for relay 1 
                    (R1), R2 and R3 respectively.
                debug:
                    Flag that will be used to print while the script is running
                    for debugging purposes
        """        
        self.debug = debug
        self.LINE_NUMBERS = pins if pins else self.LINE_NUMBERS
    
    def set_pins(self, pins: list):
        self.LINE_NUMBERS = pins

    def shoot(self):
        """
            These method will turn on the lines set earlier indefinitely.
        
        """
        import gpiod
        with gpiod.Chip(self.CHIP_NAME) as chip:
            lines = chip.get_lines(self.LINE_NUMBERS)
            lines.request(consumer='LiDRON', type=gpiod.LINE_REQ_DIR_OUT)
            if self.debug:
                print(f"Pins: {self.LINE_NUMBERS} are now controlled by this script .")
            try:                
                lines.set_values([1] * len(self.LINE_NUMBERS))
            except KeyboardInterrupt: #this exception is called when pressing CTRL+C
                lines.set_values([0] * len(self.LINE_NUMBERS))
                if self.debug:
                    print("\nExiting the script.")
    
    def stop_shooting(self):
        """
            These method will turn on the lines set earlier indefinitely.
        
        """
        import gpiod

        with gpiod.Chip(self.CHIP_NAME) as chip:
            lines = chip.get_lines(self.LINE_NUMBERS)
            lines.request(consumer='LiDRON', type=gpiod.LINE_REQ_DIR_OUT)
            if self.debug:
                print(f"Pins: {self.LINE_NUMBERS} are now controlled by this script .")

            try:                
                lines.set_values([0] * len(self.LINE_NUMBERS))
            except KeyboardInterrupt: #this exception is called when pressing CTRL+C
                lines.set_values([0] * len(self.LINE_NUMBERS))
                if self.debug:
                    print("\nExiting the script.")
    


if __name__ == "__main__":
    water_shooter = Water_pump()
    water_shooter.shoot()
    water_shooter.stop_shooting()
    
