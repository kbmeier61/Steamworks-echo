package org.usfirst.frc.team1626.robot;

public class Toggle {
	
	boolean toggleState;
	boolean prevButtonState;
	
	public Toggle() {
		toggleState=false;
		prevButtonState=false;
	}
	
	public boolean risingEdge(boolean state) {
		boolean rv = state && ! prevButtonState;
		prevButtonState = state;
		return rv;
	}
	
	public boolean fallingEdge(boolean state) {
		boolean rv = !state && prevButtonState;
		prevButtonState = state;
		return rv;
	}
	
	public boolean setState(boolean state) {
		if (risingEdge(state)) {
			toggle();
		}
		return toggleState;
	}
	
	private boolean toggle() {
		if (toggleState) {
			toggleState=false;
		} else {
			toggleState = true;
		}
		
		return toggleState;
	}
	
	public boolean getState() {
		return toggleState;
	}

}
