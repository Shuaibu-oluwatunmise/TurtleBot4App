import React, { useEffect, useRef, useState } from 'react';
import {
  View,
  Animated,
  Image,
  Text,
  StyleSheet,
  Dimensions,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

function quaternionToYaw(q: { x: number; y: number; z: number; w: number }): number {
  const { x, y, z, w } = q;
  return Math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

export default function MapViewBox({ slamRunning }: { slamRunning: boolean }) {
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const [imageData, setImageData] = useState<string | null>(null);
  const [mapInfo, setMapInfo] = useState<any>(null);
  const [robotPosition, setRobotPosition] = useState<{ x: number; y: number } | null>(null);

  const socketRef = useRef<WebSocket | null>(null);
  const lastYaw = useRef(0);
  const animatedYaw = useRef(new Animated.Value(0)).current;
  const panX = useRef(new Animated.Value(0)).current;
  const panY = useRef(new Animated.Value(0)).current;

  const { width: screenWidth } = Dimensions.get('window');
  const mapSize = screenWidth - 40;

  useEffect(() => {
    const fetchIp = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) setRobotIp(ip);
    };
    fetchIp();
  }, []);

  useEffect(() => {
    if (!robotIp || !slamRunning) {
      setImageData(null);
      setRobotPosition(null);
      return;
    }

    socketRef.current = new WebSocket(`ws://${robotIp}:9091`);
    socketRef.current.onmessage = (event) => {
      try {
        const parsed = JSON.parse(event.data);
        if (parsed.type === 'map') {
          setImageData(parsed.image);
          setMapInfo(parsed.info);
        }
      } catch (e) {
        console.warn('📡 Failed to parse map data:', e);
      }
    };

    return () => socketRef.current?.close();
  }, [robotIp, slamRunning]);

  useEffect(() => {
    if (!robotIp || !slamRunning) return;

    const odomSocket = new WebSocket(`ws://${robotIp}:9093`);
    odomSocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        const pos = data?.pose?.pose?.position;
        const ori = data?.pose?.pose?.orientation;
        if (pos && ori) {
          setRobotPosition({ x: pos.x, y: pos.y });

          // Handle rotation
          const newYaw = quaternionToYaw(ori);
          let delta = newYaw - lastYaw.current;
          if (delta > Math.PI) delta -= 2 * Math.PI;
          if (delta < -Math.PI) delta += 2 * Math.PI;
          const targetYaw = lastYaw.current + delta;
          lastYaw.current = targetYaw;

          Animated.timing(animatedYaw, {
            toValue: targetYaw,
            duration: 300,
            useNativeDriver: true,
          }).start();
        }
      } catch (e) {
        console.warn('📡 Failed to parse odom data:', e);
      }
    };

    return () => odomSocket.close();
  }, [robotIp, slamRunning]);

  const getTransforms = () => {
    if (!mapInfo || !robotPosition) return { icon: {}, origin: {}, px: 0, py: 0 };

    const { width: mapWidth, height: mapHeight, resolution, origin } = mapInfo;
    const originPos = origin?.position || { x: 0, y: 0 };
    const pixelsPerCell = mapSize / mapWidth;

    const mapX = (robotPosition.x - originPos.x) / resolution;
    const mapY = (robotPosition.y - originPos.y) / resolution;
    const flippedY = mapHeight - mapY;

    const px = mapX * pixelsPerCell;
    const py = flippedY * pixelsPerCell;

    const iconSize = (0.4 / resolution) * pixelsPerCell;
    const iconStyle = {
      position: 'absolute' as const,
      left: px - iconSize / 2,
      top: py - iconSize / 2,
      width: iconSize,
      height: iconSize,
      zIndex: 10,
    };

    const centerOffset = mapSize / 2;

    Animated.timing(panX, {
      toValue: centerOffset - px,
      duration: 200,
      useNativeDriver: true,
    }).start();
    Animated.timing(panY, {
      toValue: centerOffset - py,
      duration: 200,
      useNativeDriver: true,
    }).start();

    return {
      icon: iconStyle,
      origin: {
        position: 'absolute' as const,
        left: ((0 - originPos.x) / resolution) * pixelsPerCell - 6,
        top: (mapHeight - ((0 - originPos.y) / resolution)) * pixelsPerCell - 6,
        width: 12,
        height: 12,
        backgroundColor: 'magenta',
        borderRadius: 6,
        zIndex: 1000,
      },
      px,
      py,
    };
  };

  const interpolatedRotate = animatedYaw.interpolate({
    inputRange: [-Math.PI, Math.PI],
    outputRange: ['180deg', '-180deg'],
  });

  const { icon, origin, px, py } = getTransforms();

  return (
    <View style={styles.wrapper}>
      {slamRunning && imageData ? (
        <View style={styles.mapContainer}>
          <Animated.View
            style={{
              width: mapSize,
              height: mapSize,
              transform: [{ translateX: panX }, { translateY: panY }],
            }}
          >
            <Image
              source={{ uri: `data:image/png;base64,${imageData}` }}
              style={[styles.mapImage, { transform: [{ scaleY: -1 }] }]}
              resizeMode="contain"
            />
            <Animated.Image
              source={require('@/assets/images/turtle.png')}
              style={[icon, { transform: [{ rotate: interpolatedRotate }, { rotate: '180deg' }] }]}
            />
            <View
              style={{
                position: 'absolute',
                left: px - 2,
                top: py - 2,
                width: 4,
                height: 4,
                backgroundColor: 'red',
                borderRadius: 2,
                zIndex: 100,
              }}
            />
            <View style={origin} />
          </Animated.View>
        </View>
      ) : (
        <View style={styles.placeholderContainer}>
          <Text style={styles.placeholderText}>🕓 Waiting for SLAM...</Text>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    flex: 1,
    backgroundColor: '#111',
    justifyContent: 'center',
    alignItems: 'center',
    overflow: 'hidden',
  },
  mapContainer: {
    width: '100%',
    aspectRatio: 1,
    overflow: 'hidden',
    alignItems: 'center',
    justifyContent: 'center',
  },
  mapImage: {
    width: '100%',
    height: '100%',
    position: 'absolute',
  },
  placeholderContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  placeholderText: {
    color: '#888',
    fontSize: 16,
    fontStyle: 'italic',
  },
});