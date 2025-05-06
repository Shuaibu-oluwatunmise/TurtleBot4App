import React from 'react';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import LandingScreen from '@/features/Landing/LandingScreen';
import MenuScreen from '@/features/Menu/MenuScreen';
import BatteryScreen from '@/features/Battery/BatteryScreen';
import TeleopScreen from './features/Teleop/TeleopScreen';
import 'react-native-gesture-handler';


export type RootStackParamList = {
  Landing: undefined;
  Menu: undefined;
  Battery: undefined;
  Teleop: undefined;
};

const Stack = createNativeStackNavigator<RootStackParamList>();

export default function App() {
  return (
    <GestureHandlerRootView style={{ flex: 1 }}>
      <NavigationContainer>
        <Stack.Navigator initialRouteName="Landing" screenOptions={{ headerShown: false }}>
          <Stack.Screen name="Landing" component={LandingScreen} />
          <Stack.Screen name="Menu" component={MenuScreen} />
          <Stack.Screen name="Battery" component={BatteryScreen} />
          <Stack.Screen name="Teleop" component={TeleopScreen} />
        </Stack.Navigator>
      </NavigationContainer>
    </GestureHandlerRootView>
  );
}
