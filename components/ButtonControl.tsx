import React, { useRef, useEffect } from 'react';
import { View, Pressable, Text, StyleSheet } from 'react-native';

interface ButtonControlProps {
  linearSpeed: number;
  angularSpeed: number;
  sendCommand: (linear: number, angular: number) => void;
  enabled: boolean;
}

export default function ButtonControl({ linearSpeed, angularSpeed, sendCommand, enabled }: ButtonControlProps) {
  const moveInterval = useRef<NodeJS.Timeout | null>(null);

  const handlePressIn = (linear: number, angular: number) => {
    moveInterval.current = setInterval(() => sendCommand(linear, angular), 100);
  };

  const handlePressOut = () => {
    if (moveInterval.current) clearInterval(moveInterval.current);
    sendCommand(0, 0);
  };

  return (
    <View style={styles.controlPad}>
      <View style={styles.buttonRow}>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(linearSpeed, angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>↖️</Text>
        </Pressable>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(linearSpeed, 0)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>⬆️</Text>
        </Pressable>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(linearSpeed, -angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>↗️</Text>
        </Pressable>
      </View>

      <View style={styles.buttonRow}>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(0, angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>⬅️</Text>
        </Pressable>
        <Pressable style={[styles.arrowButton, { backgroundColor: '#505050' }]} onPressIn={() => sendCommand(0, 0)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>⛔</Text>
        </Pressable>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(0, -angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>➡️</Text>
        </Pressable>
      </View>

      <View style={styles.buttonRow}>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(-linearSpeed, angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>↙️</Text>
        </Pressable>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(-linearSpeed, 0)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>⬇️</Text>
        </Pressable>
        <Pressable style={styles.arrowButton} onPressIn={() => handlePressIn(-linearSpeed, -angularSpeed)} onPressOut={handlePressOut}>
          <Text style={styles.arrowText}>↘️</Text>
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  controlPad: {
    justifyContent: 'center',
    alignItems: 'center',
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-evenly',
    width: '100%',
  },
  arrowButton: {
    width: 70,
    height: 70,
    backgroundColor: '#333',
    borderRadius: 16,
    justifyContent: 'center',
    alignItems: 'center',
    margin: 10,
  },
  arrowText: {
    fontSize: 30,
    color: '#fff',
  },
});
