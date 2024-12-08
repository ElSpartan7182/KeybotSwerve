package com.team5716.lib.KeybotPreferences;

public abstract class KeybotPreferences {
    private static boolean useDefaults = false;

    protected String m_name;
    
    public static void useDefaults() {
        useDefaults = true;
    }
    
    public static void usePreferences() {
        useDefaults = false;
    }
    public static boolean isUsingDefaults() {
        return useDefaults;
    }
}