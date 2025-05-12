import React, { useEffect, useRef } from 'react';
import { View, StyleSheet, Text } from 'react-native';
import Canvas from 'react-native-canvas';
import { throttle } from 'lodash';

export default function MapCanvas({ map }) {
  const canvasRef = useRef(null);

  const throttledDraw = useRef(throttle(async (canvas, map) => {
    const { width, height } = map.info;
    const data = map.data;
    const scale = 2;

    console.log("🗺️ Map received:", width, 'x', height);
    console.log("🧪 Sample data:", data.slice(0, 10));

    const ctx = await canvas.getContext('2d');
    if (!ctx) {
      console.warn('❗ Canvas context not available');
      return;
    }

    canvas.width = width * scale;
    canvas.height = height * scale;

    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      let color;
      if (val === -1) color = '#808080';       // Unknown - gray
      else if (val === 0) color = '#FFFFFF';   // Free - white
      else if (val === 100) color = '#000000'; // Occupied - black
      else color = '#FF0000';                  // Unexpected - red

      const x = i % width;
      const y = height - 1 - Math.floor(i / width); // Flip Y

      ctx.fillStyle = color;
      ctx.fillRect(x * scale, y * scale, scale, scale);
    }
  }, 500)).current; // Throttle drawing to every 500ms

  useEffect(() => {
    if (canvasRef.current && map && map.info && map.data) {
      throttledDraw(canvasRef.current, map);
    }
  }, [map]);

  return (
    <View style={styles.canvasWrapper}>
      {map ? (
        <Canvas
          ref={canvasRef}
          style={styles.canvas}
        />
      ) : (
        <Text style={styles.placeholder}>Waiting for map data...</Text>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  canvasWrapper: {
    width: '100%',
    aspectRatio: 1.5,
    backgroundColor: '#2a2a2a',
    borderRadius: 12,
    overflow: 'hidden',
    borderColor: '#00ffcc',
    borderWidth: 1,
    marginTop: 20,
  },
  canvas: {
    width: '100%',
    height: '100%',
  },
  placeholder: {
    color: '#888',
    textAlign: 'center',
    marginTop: 10,
  },
});
