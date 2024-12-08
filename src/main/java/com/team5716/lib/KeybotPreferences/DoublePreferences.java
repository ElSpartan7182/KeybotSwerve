package com.team5716.lib.KeybotPreferences;

import edu.wpi.first.wpilibj.Preferences;

public class DoublePreferences extends KeybotPreferences {
	private double m_defaultValue;

	public DoublePreferences(String name, double defaultValue) {
		m_name = name;
		m_defaultValue = defaultValue;
	}
	public double getValue() {
		if (isUsingDefaults()) {
		    return m_defaultValue;
		}
		return Preferences.getDouble(m_name, m_defaultValue);
	}
}