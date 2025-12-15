#!/usr/bin/env python3

"""
Brief description.
"""

class SampleClass:
    """
    This is a sample class

    Attributes:
        name (str): Your name
    Methods:
        welcome_print(): Prints a welcome message
    """
    def __init__(self, name):
        """
        Constructs all the necessary attributes for the class

        Args:
            name (str): Your name
        """
        self.name = name

    def welcome_print(self):
        """This function prints a welcome message
        """
        print("Welcome " + self.name + " !!!")


def add_two_things(self, num1, num2=10):
    """
    This is a function to add two numbers

    Args:
        num1 (int): A number
        num2 (int): Another number, default 10

    Returns:
        int: the sum of two numbers
    """
    return num1 + num2
    