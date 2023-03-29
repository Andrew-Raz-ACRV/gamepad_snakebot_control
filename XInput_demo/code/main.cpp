//------------------------------------------------------------
// C++ Xbox 360 gamepad Input using XInput
// Demo code by Lawrence M
//------------------------------------------------------------
// This is a demo program to demonstrate the 'Gamepad' class
// (based on the code from my 3-part tutorial).
//
// All code in this program (including 'Gamepad' class) was
// written and tested on a Windows 7 PC using Visual Studio
// 2013. A Microsoft (USB wired) Xbox 360 controller was used
// to test the gamepad code.
//------------------------------------------------------------

#include <iostream>

#include "Gamepad.h"

using namespace std;

bool running;    // Used to break the loop
Gamepad gamepad; // Gamepad instance


// This example checks input on all gamepad buttons (Guide/logo
// button not supported). For example code on using the triggers
// and thumbsticks, please refer to the tutorial.
void TestGamepad()
{
	// GetButtonDown only returns true on the frame it was first pressed.
	if (gamepad.GetButtonDown(xButtons.A))
	{
		cout << " Button [A] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.X))
	{
		cout << " Button [X] pressed" << endl;
	}

	// GetButtonPressed will keep returning true until the button is released.
	if (gamepad.GetButtonPressed(xButtons.R_Shoulder))
	{
		cout << " Button [Y] held, see how this doesn't appear just once?" << endl;
	}

	// Check the D-Pad buttons
	if (gamepad.GetButtonDown(xButtons.DPad_Up))
	{
		cout << " Button [DPad Up] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.DPad_Down))
	{
		cout << " Button [DPad Down] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.DPad_Left))
	{
		cout << " Button [DPad Left] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.DPad_Right))
	{
		cout << " Button [DPad Right] pressed" << endl;
	}

	// Check the Shoulder ('bumper') buttons
	if (gamepad.GetButtonDown(xButtons.L_Shoulder))
	{
		cout << " Button [L Bumper] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.R_Shoulder))
	{
		cout << " Button [R Bumper] pressed" << endl;
	}

	// Check the BACK and START buttons
	if (gamepad.GetButtonDown(xButtons.Back))
	{
		cout << " Button [BACK] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.Start))
	{
		cout << " Button [START] pressed" << endl;
	}

	// Check the Thumbstick buttons (press in the thumbstick)
	if (gamepad.GetButtonDown(xButtons.L_Thumbstick))
	{
		cout << " Button [L STICK] pressed" << endl;
	}

	if (gamepad.GetButtonDown(xButtons.R_Thumbstick))
	{
		cout << " Button [R STICK] pressed" << endl;
	}
}

int main()
{
	// Set up
	running = true;
	gamepad = Gamepad(1); // Set gamepad ID to 1

	cout << " --------------------------------------------" << endl;
	cout << "          Xbox 360 Gamepad INPUT TEST        " << endl;
	cout << " --------------------------------------------" << endl << endl;

	cout << " Use gamepad buttons to test gamepad input." << endl;
	cout << " To quit, press the [B] button." << endl;
	cout << " --------------------------------------------" << endl << endl;

	// Pretend game loop, repeat until 'B' is pressed
	do
	{
		gamepad.Update(); // Update gamepad

		if (gamepad.Connected())
		{
			// Run gamepad input test
			TestGamepad();

			// Pressing B quits the program
			if (gamepad.GetButtonPressed(xButtons.B))
				running = false;
		}


		gamepad.Refresh(); // Update gamepad for next cycle
	}
	while (running);
	
	return 0;
}