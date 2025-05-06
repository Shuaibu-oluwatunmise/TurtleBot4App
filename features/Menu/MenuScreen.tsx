import React from 'react';
import { View, Text, Pressable, StyleSheet, Alert } from 'react-native';
import { useNavigation } from '@react-navigation/native';
import { NativeStackNavigationProp } from '@react-navigation/native-stack';
import { RootStackParamList } from '@/App';

type NavigationProp = NativeStackNavigationProp<RootStackParamList, 'Menu'>;

export default function MenuScreen() {
  const navigation = useNavigation<NavigationProp>();

  const handleComingSoon = () => {
    Alert.alert('Coming Soon 🚧', 'This feature is under development.');
  };

  return (
    <View style={styles.container}>
      <Text style={styles.title}>🤖 TurtleBot Actions</Text>

      <View style={styles.buttonGrid}>
        <Pressable style={styles.button} onPress={() => navigation.navigate('Battery')}>
          <Text style={styles.buttonText}>🔋 Battery</Text>
        </Pressable>

        <Pressable style={styles.button} onPress={() => navigation.navigate('Teleop')}>
          <Text style={styles.buttonText}>📡 Teleop</Text>
        </Pressable>

        <Pressable style={styles.button} onPress={handleComingSoon}>
          <Text style={styles.buttonText}>📈 Diagnostics</Text>
        </Pressable>

        <Pressable style={styles.button} onPress={handleComingSoon}>
          <Text style={styles.buttonText}>📷 Camera</Text>
        </Pressable>

        <Pressable style={styles.button} onPress={handleComingSoon}>
          <Text style={styles.buttonText}>🛠️ Settings</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1, paddingTop: 80, alignItems: 'center', backgroundColor: '#1e1e1e' },
  title: { fontSize: 24, color: '#00ffcc', marginBottom: 32 },
  buttonGrid: {
    width: '90%',
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  button: {
    width: '48%',
    backgroundColor: '#333',
    paddingVertical: 20,
    borderRadius: 12,
    marginBottom: 20,
    alignItems: 'center',
  },
  buttonText: { fontSize: 16, color: '#fff', textAlign: 'center' },
});
