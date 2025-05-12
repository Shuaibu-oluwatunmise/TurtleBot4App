import React from 'react';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import LandingScreen from '@/features/Landing/LandingScreen';
import IpLoginScreen from '@/features/IpLogin/IpLoginScreen';
import MenuScreen from '@/features/Menu/MenuScreen';
import BatteryScreen from '@/features/Battery/BatteryScreen';
import TeleopScreen from '@/features/Teleop/TeleopScreen';
import MapScreen from '@/features/Map/MapScreen';
import 'react-native-gesture-handler';

export type RootStackParamList = {
  Landing: undefined;
  IpLogin: undefined;
  Menu: undefined;
  Battery: undefined;
  Teleop: undefined;
  App: undefined,
};

const Stack = createNativeStackNavigator<RootStackParamList>();

export default function App() {
  return (
    <GestureHandlerRootView style={{ flex: 1 }}>
      <NavigationContainer>
        <Stack.Navigator initialRouteName="Landing" screenOptions={{ headerShown: false }}>
          <Stack.Screen name="Landing" component={LandingScreen} />
          <Stack.Screen name="IpLogin" component={IpLoginScreen} />
          <Stack.Screen name="Menu" component={MenuScreen} />
          <Stack.Screen name="Battery" component={BatteryScreen} />
          <Stack.Screen name="Teleop" component={TeleopScreen} />
          <Stack.Screen name="Map" component={MapScreen} />
        </Stack.Navigator>
      </NavigationContainer>
    </GestureHandlerRootView>
  );
}
