import React, { useRef, useState, useEffect } from 'react';
import {
  View,
  Text,
  TextInput,
  StyleSheet,
  TouchableOpacity,
  KeyboardAvoidingView,
  Platform,
  Keyboard,
  TouchableWithoutFeedback,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useNavigation } from '@react-navigation/native';

export default function IpLoginScreen() {
  const navigation = useNavigation();
  const [segments, setSegments] = useState(['', '', '', '']);
  const refs = [useRef(null), useRef(null), useRef(null), useRef(null)];

  useEffect(() => {
    const checkStoredIp = async () => {
      const storedIp = await AsyncStorage.getItem('robotIp');
      if (storedIp) {
        navigation.reset({ index: 0, routes: [{ name: 'Menu' }] });
      }
    };
    checkStoredIp();
  }, []);


  const limits = [3, 3, 1, 3];

  const handleChange = (index, value) => {
    if (value.length > limits[index]) return;
    const newSegs = [...segments];
    newSegs[index] = value;
    setSegments(newSegs);

    if (value.length === limits[index] && index < refs.length - 1) {
      refs[index + 1].current.focus();
    }
  };

  const handleLogin = async () => {
    const ip = segments.join('.');
    const ipRegex = /^(\d{1,3})\.(\d{1,3})\.(\d{1})\.(\d{1,3})$/;
    if (!ipRegex.test(ip)) {
      alert('Invalid IP address');
      return;
    }
    await AsyncStorage.setItem('robotIp', ip);
    navigation.reset({ index: 0, routes: [{ name: 'Menu' }] });
  };

  return (
    <TouchableWithoutFeedback onPress={Keyboard.dismiss} accessible={false}>
      <KeyboardAvoidingView behavior={Platform.OS === 'ios' ? 'padding' : undefined} style={styles.container}>
        <Text style={styles.title}>Enter TurtleBot IP</Text>
        <View style={styles.inputRow}>
          {segments.map((seg, idx) => (
            <React.Fragment key={idx}>
              <TextInput
                ref={refs[idx]}
                style={[styles.inputBox, seg.length === limits[idx] ? styles.filled : null]}
                keyboardType="numeric"
                value={seg}
                maxLength={limits[idx]}
                onChangeText={(text) => handleChange(idx, text)}
                autoFocus={idx === 0}
              />
              {idx < 3 && <Text style={styles.dot}>.</Text>}
            </React.Fragment>
          ))}
        </View>
        <TouchableOpacity style={styles.button} onPress={handleLogin}>
          <Text style={styles.buttonText}>Sign In</Text>
        </TouchableOpacity>
      </KeyboardAvoidingView>
    </TouchableWithoutFeedback>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1e1e1e',
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 20,
  },
  title: {
    color: '#00ffcc',
    fontSize: 20,
    marginBottom: 30,
    fontWeight: 'bold',
  },
  inputRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 40,
  },
  inputBox: {
    width: 60,
    height: 60,
    borderRadius: 12,
    borderWidth: 2,
    borderColor: '#00ffcc',
    color: '#fff',
    textAlign: 'center',
    fontSize: 20,
    marginHorizontal: 4,
    backgroundColor: '#040404',
    shadowColor: '#00ffcc',
    shadowOffset: { width: 0, height: 0 },
    shadowOpacity: 0,
    shadowRadius: 0,
  },
  filled: {
    shadowOpacity: 0.4,
    shadowRadius: 10,
  },
  dot: {
    color: '#00ffcc',
    fontSize: 24,
    fontWeight: 'bold',
    marginHorizontal: 2,
  },
  button: {
    backgroundColor: '#00ffcc',
    paddingVertical: 14,
    paddingHorizontal: 40,
    borderRadius: 12,
  },
  buttonText: {
    color: '#000',
    fontSize: 16,
    fontWeight: 'bold',
  },
});