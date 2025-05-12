import React, { useEffect, useRef } from 'react';
import { View, StyleSheet, Text, PanResponder, Animated } from 'react-native';

interface Props {
  ws: React.MutableRefObject<WebSocket | null>;
  linearSpeed: number;
  angularSpeed: number;
  enabled: boolean;
}

export default function JoystickControl({ ws, linearSpeed, angularSpeed, enabled }: Props) {
  const moveInterval = useRef<NodeJS.Timeout | null>(null);
  const latestCommand = useRef({ linear: 0, angular: 0 });
  const pan = useRef(new Animated.ValueXY()).current;

  const sendCommand = (linear: number, angular: number) => {
    if (!enabled) return;
    if (!ws.current || ws.current.readyState !== WebSocket.OPEN) return;
    ws.current.send(
      JSON.stringify({
        op: 'publish',
        topic: '/cmd_vel',
        msg: {
          header: {
            stamp: { sec: 0, nanosec: 0 },
            frame_id: '',
          },
          twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
          },
        },
      })
    );
  };

  useEffect(() => {
    moveInterval.current = setInterval(() => {
      sendCommand(latestCommand.current.linear, latestCommand.current.angular);
    }, 100);

    return () => {
      if (moveInterval.current) clearInterval(moveInterval.current);
      sendCommand(0, 0);
    };
  }, [enabled]);

  const panResponder = useRef(
    PanResponder.create({
      onMoveShouldSetPanResponder: () => enabled,
      onPanResponderMove: (_, gestureState) => {
        if (!enabled) return;

        const { dx, dy } = gestureState;
        pan.setValue({ x: dx, y: dy });

        const maxDistance = 100;
        const clampedX = Math.max(-maxDistance, Math.min(dx, maxDistance));
        const clampedY = Math.max(-maxDistance, Math.min(dy, maxDistance));

        const linear = -clampedY / maxDistance * linearSpeed;
        const angular = -clampedX / maxDistance * angularSpeed;

        latestCommand.current = { linear, angular };
      },
      onPanResponderRelease: () => {
        Animated.spring(pan, { toValue: { x: 0, y: 0 }, useNativeDriver: false }).start();
        latestCommand.current = { linear: 0, angular: 0 };
      },
    })
  ).current;

  return (
    <View style={styles.container}>
      <View style={styles.joystickBase}>
        <Animated.View
          {...panResponder.panHandlers}
          style={[styles.stick, { transform: pan.getTranslateTransform() }]}
        />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    alignItems: 'center',
    justifyContent: 'center',
    paddingTop: 20,
  },
  joystickBase: {
    width: 200,
    height: 200,
    borderRadius: 100,
    backgroundColor: '#333',
    justifyContent: 'center',
    alignItems: 'center',
  },
  stick: {
    width: 80,
    height: 80,
    borderRadius: 40,
    backgroundColor: '#00ffcc',
    position: 'absolute',
  },
});
