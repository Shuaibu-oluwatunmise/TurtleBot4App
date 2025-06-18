import React, { useEffect, useRef } from 'react';
import {
  View,
  FlatList,
  Image,
  StyleSheet,
  Dimensions,
  Text,
  SafeAreaView,
  TouchableOpacity,
} from 'react-native';
import { GestureDetector, Gesture } from 'react-native-gesture-handler';
import Animated, {
  useAnimatedStyle,
  useSharedValue,
  withSpring,
  runOnJS,
} from 'react-native-reanimated';

export default function FullGalleryScreen({ route, navigation }) {
  const { maps, initialIndex } = route.params;
  const flatListRef = useRef(null);
  const { width, height } = Dimensions.get('window');
  const translateY = useSharedValue(0);

  // 🧲 Pan gesture for swipe-down to dismiss
  const panGesture = Gesture.Pan()
    .onUpdate((e) => {
      if (e.translationY > 0) {
        translateY.value = e.translationY;
      }
    })
    .onEnd((e) => {
      if (e.translationY > 150) {
        runOnJS(navigation.goBack)();
      } else {
        translateY.value = withSpring(0);
      }
    });

  const animatedStyle = useAnimatedStyle(() => ({
    transform: [{ translateY: translateY.value }],
  }));

  return (
    <SafeAreaView style={styles.safeArea}>
      <TouchableOpacity onPress={() => navigation.goBack()} style={styles.backButton}>
        <Text style={styles.backText}>← Back</Text>
      </TouchableOpacity>

      <GestureDetector gesture={panGesture}>
        <Animated.View style={[styles.animatedWrapper, animatedStyle]}>
          <FlatList
            ref={flatListRef}
            data={maps}
            horizontal
            pagingEnabled
            initialScrollIndex={initialIndex}
            getItemLayout={(data, index) => ({
              length: width,
              offset: width * index,
              index,
            })}
            keyExtractor={(item) => item.filename}
            renderItem={({ item }) => (
              <View style={[styles.page, { width, height }]}>
                <Image
                  source={{ uri: `data:image/png;base64,${item.base64}` }}
                  style={styles.image}
                  resizeMode="contain"
                />
                <Text style={styles.caption}>{item.filename}</Text>
              </View>
            )}
          />
        </Animated.View>
      </GestureDetector>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#000',
  },
  backButton: {
    position: 'absolute',
    top: 20,
    left: 16,
    zIndex: 10,
    padding: 8,
  },
  backText: {
    color: '#00ffcc',
    fontSize: 18,
  },
  animatedWrapper: {
    flex: 1,
  },
  page: {
    justifyContent: 'center',
    alignItems: 'center',
  },
  image: {
    width: '100%',
    height: '85%',
  },
  caption: {
    color: '#aaa',
    marginTop: 10,
    fontSize: 14,
    textAlign: 'center',
  },
});
