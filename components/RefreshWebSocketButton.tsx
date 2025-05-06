import React, { useRef, useEffect } from 'react';
import { Pressable, Animated, StyleSheet } from 'react-native';
import { Ionicons } from '@expo/vector-icons';

type Props = {
  onPress: () => void;
};

export default function RefreshWebSocketButton({ onPress }: Props) {
  const spinAnim = useRef(new Animated.Value(0)).current;

  const spin = () => {
    spinAnim.setValue(0);
    Animated.timing(spinAnim, {
      toValue: 1,
      duration: 600,
      useNativeDriver: true,
    }).start();
  };

  const spinInterpolate = spinAnim.interpolate({
    inputRange: [0, 1],
    outputRange: ['0deg', '360deg'],
  });

  const handlePress = () => {
    spin();
    onPress();
  };

  return (
    <Pressable onPress={handlePress} style={styles.button}>
      <Animated.View style={{ transform: [{ rotate: spinInterpolate }] }}>
        <Ionicons name="refresh" size={24} color="#00ffcc" />
      </Animated.View>
    </Pressable>
  );
}

const styles = StyleSheet.create({
  button: {
    padding: 8,
  },
});
