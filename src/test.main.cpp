// ===================== TEST MODE CONFIGURATION =====================
// This file contains test mode setup for HVAC system testing
// Copy a scenario line from here into main.cpp setup() to enable testing
// Only ONE scenario should be enabled at a time

// To use:
// 1. Copy ONE enableTestMode() line below
// 2. Paste it in main.cpp setup() in the TEST MODE SETUP section
// 3. Upload to ESP32

// ===================== AVAILABLE TEST SCENARIOS =====================

// Normal operation scenarios:
// enableTestMode(TestScenario::Normal_Cooling);
// enableTestMode(TestScenario::Normal_Heating);
// enableTestMode(TestScenario::System_Off);

// Fault scenarios:
// enableTestMode(TestScenario::Low_Refrigerant_Charge);        /* MODE=Mismatch ; may want to think of a way to make mismatch isolated to EVAP/COND OFF */
// enableTestMode(TestScenario::Dirty_Condenser);               
// enableTestMode(TestScenario::Dirty_Evaporator);              
// enableTestMode(TestScenario::Restriction_TXV);               /* DIAG=RESTRICTION; FAULT_RESTRICTION_PATTERN */
// enableTestMode(TestScenario::Suction_Line_Freezing);         /* Add diag=FREEZING_AirHandler if deltaT pattern matches; FAULT_FREEZING_PATTERN */
// enableTestMode(TestScenario::Static_Not_Equalizing);
// enableTestMode(TestScenario::Weak_Airflow);                  /* MODE=Mismatch ; may want to think of a way to make mismatch isolated to EVAP/COND OFF */
// enableTestMode(TestScenario::Multiple_Faults);
// enableTestMode(TestScenario::All_Sensors_Fault);             /* All sensors reading NAN; diag=SENSOR_FAULT; FAULT_ALL_SENSORS_FAULT_PATTERN */
// enableTestMode(TestScenario::Custom);

// Utility function:
// printTestScenarioList();  // Show all available scenarios in serial output

// ===================== NOTES =====================
// - Test mode bypasses real sensor hardware initialization
// - Data comes from hvac_test_data.h
// - Useful for debugging diagnostics without physical sensors
// - See TEST_MODE_GUIDE.md for detailed documentation
