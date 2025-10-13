/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "usb_audio_interface.h"

#ifdef AUDIO_INTERFACE

///////////////////////////
// Interface
///////////////////////////

#if OPENAUDIO
	#include <AudioStream_F32.h>
	#define AudioStream_CLASS AudioStream_F32
	#define sample_TYPE float32_t
#else
	#include <AudioStream.h>
	#define AudioStream_CLASS AudioStream
	#define sample_TYPE int16_t
#endif

template <typename StreamClass>
class AudioUSB_Base:
	public StreamClass
{
protected:
#if OPENAUDIO
	typedef ::audio_block_f32_t audio_block_t;
	static AudioUSB_Base::audio_block_t *allocate() { return StreamClass::allocate_f32(); }
	static void release(AudioUSB_Base::audio_block_t *block) { StreamClass::release(block); }
	audio_block_t *receiveReadOnly(unsigned int index = 0) { return StreamClass::receiveReadOnly_f32(index); }
	audio_block_t *receiveWritable(unsigned int index = 0) { return StreamClass::receiveWritable_f32(index); }
	void transmit(AudioUSB_Base::audio_block_t *block, unsigned char index = 0) { StreamClass::transmit(block, index); }
	static int blocklength(const AudioUSB_Base::audio_block_t *block) { return block->length; }
#else
	typedef ::audio_block_t audio_block_t;
	static int blocklength(const AudioUSB_Base::audio_block_t *) { return AUDIO_BLOCK_SAMPLES; }
#endif
public:
	AudioUSB_Base(unsigned char ninput, AudioUSB_Base::audio_block_t **iqueue): StreamClass(ninput, iqueue) {}
};

template <typename StreamClass>
class AudioInputUSB_Proto:
	public AudioUSB_Base<StreamClass>
{
public:	
	AudioInputUSB_Proto(float kp = 400.f, float ki = .2f):
		AudioUSB_Base<StreamClass>(0, NULL),
		_usbInterface(setBlockQuiet, releaseBlock, allocateBlock, areBlocksReady, copy_to_buffers, kp, ki)
	{
		for(uint16_t i = 0; i < USBAudioInInterface::ringRxBufferSize; i++)
			for(uint16_t j = 0; j < USB_AUDIO_MAX_NO_CHANNELS; j++)
				rxBuffer[i][j] = NULL;

		_usbInterface.begin();
	}

	virtual void update()
	{
		int16_t bIdx = -1;
		uint16_t noChannels;
		_usbInterface.update(bIdx, noChannels);
		if(bIdx != -1) {
			for(uint16_t i = 0; i < noChannels; i++) {
				AudioUSB_Base<StreamClass>::transmit(rxBuffer[bIdx][i], i);
				AudioUSB_Base<StreamClass>::release(rxBuffer[bIdx][i]);
				rxBuffer[bIdx][i] = NULL;
			}
			_usbInterface.incrementBufferIndex();
		}
	}

	void begin() {}

	float getBufferedSamples() const
	{
		return _usbInterface.getBufferedSamples();
	}

	float getBufferedSamplesSmooth() const
	{
		return _usbInterface.getBufferedSamplesSmooth();
	}

	float getRequestedSamplingFrequ() const
	{
		return _usbInterface.getRequestedSamplingFrequ();
	}

	float getActualBIntervalUs() const
	{
		return _usbInterface.getActualBIntervalUs();
	}

	USBAudioInInterface::Status getStatus() const
	{
		return _usbInterface.getStatus();
	}

	float volume()
	{
		return _usbInterface.volume();
	}

private:
	static void copy_to_buffers(const uint8_t *src, uint16_t bIdx, uint16_t noChannels, unsigned int count, unsigned int len)
	{
		for(uint32_t i = 0; i < len; i++)
			for(uint16_t j = 0; j < noChannels; j++) {
	#if USB_AUDIO_FORMAT == 1 // PCM
		#if AUDIO_SUBSLOT_SIZE>=2 && AUDIO_SUBSLOT_SIZE<=4
				// USB PCM data is always signed
			#if OPENAUDIO
				union {
					int32_t i32;
					uint8_t u8[4];
				} tmp;
				constexpr auto scale = 1<<(sizeof(tmp.i32)*8-1);
				tmp.i32 = 0;
				for(int k = 0; k < AUDIO_SUBSLOT_SIZE; ++k, ++src)
					tmp.u8[k+(4-AUDIO_SUBSLOT_SIZE)] = *src;
				// convert to float
				rxBuffer[bIdx][j]->data[count+i] = tmp.i32*(1.f/float32_t(scale));
			#else
				src += (AUDIO_SUBSLOT_SIZE-2); // eventually ignore low PCM bytes (with loss of precision)
				const int16_t *src16Bit = (const int16_t *)src;
				rxBuffer[bIdx][j]->data[count+i] = *src16Bit;
				src += 2;
			#endif
		#else
			#error AUDIO_SUBSLOT_SIZE invalid
		#endif
	#elif USB_AUDIO_FORMAT == 4 // IEEE_FLOAT
			#if OPENAUDIO
				rxBuffer[bIdx][j]->data[count+i] = *(const float32_t *)(src);
			#else
				constexpr auto scale = 1<<(sizeof(int16_t)*8-1);
				constexpr auto fmin = -1.f;
				constexpr auto fmax = float((scale-1.)/scale);
				rxBuffer[bIdx][j]->data[count+i] = int16_t(min(max(*src, fmin), fmax)*scale);
			#endif
				src += 4;
	#else
		#error USB_AUDIO_FORMAT invalid
	#endif
			}
	}

	static bool setBlockQuiet(uint16_t bIdx, uint16_t channel)
	{        
		if(!rxBuffer[bIdx][channel])
			rxBuffer[bIdx][channel] = AudioUSB_Base<StreamClass>::allocate();

		if(rxBuffer[bIdx][channel]) {
			memset(rxBuffer[bIdx][channel]->data, 0, AUDIO_BLOCK_SAMPLES*sizeof(rxBuffer[bIdx][channel]->data[0]));
			return true;
		}
		return false;
	}

	static void releaseBlock(uint16_t bIdx, uint16_t channel)
	{        
		if(rxBuffer[bIdx][channel]) {
			AudioUSB_Base<StreamClass>::release(rxBuffer[bIdx][channel]);
			rxBuffer[bIdx][channel] = NULL;
		}
	}

	static bool allocateBlock(uint16_t bIdx, uint16_t channel)
	{        
		if(!rxBuffer[bIdx][channel]) {
			rxBuffer[bIdx][channel] = AudioUSB_Base<StreamClass>::allocate();
		}
		return rxBuffer[bIdx][channel] != NULL;
	}

	static bool areBlocksReady(uint16_t bIdx, uint16_t noChannels)
	{
		for(uint16_t i = 0; i < noChannels; i++) {
			if(!rxBuffer[bIdx][i]) return false;
		}
		return true;
	}

    static typename AudioUSB_Base<StreamClass>::audio_block_t* rxBuffer[USBAudioInInterface::ringRxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];
    USBAudioInInterface _usbInterface;
};

template <typename StreamClass>
typename AudioUSB_Base<StreamClass>::audio_block_t* AudioInputUSB_Proto<StreamClass>::rxBuffer[USBAudioInInterface::ringRxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];



template <typename StreamClass>
class AudioOutputUSB_Proto:
	public AudioUSB_Base<StreamClass>
{
public:
	AudioOutputUSB_Proto(int nch = 0):
		AudioUSB_Base<StreamClass>(nch?nch:USB_AUDIO_MAX_NO_CHANNELS, inputQueueArray),
		_usbInterface(releaseBlocks, isBlockReady, copy_from_buffers)
	{
		begin();
		_usbInterface.begin();
	}

	virtual void update()
	{
		int16_t bIdx = -1;
		uint16_t noChannels;
		_usbInterface.update(bIdx, noChannels);

		if(bIdx < 0) {
			//_usbInterface is not running for some reason
			for (uint16_t i = 0; i < USB_AUDIO_MAX_NO_CHANNELS; i++){
				typename AudioUSB_Base<StreamClass>::audio_block_t* b = AudioUSB_Base<StreamClass>::receiveReadOnly(i);
				if(b)
					AudioUSB_Base<StreamClass>::release(b);
			}
		}

		for(uint16_t i = 0; i < noChannels; i++) {
			if(txBuffer[bIdx][i])
				AudioUSB_Base<StreamClass>::release(txBuffer[bIdx][i]);

			txBuffer[bIdx][i] = AudioUSB_Base<StreamClass>::receiveReadOnly(i);
			if(!txBuffer[bIdx][i]) {
				if(!txBuffer[bIdx][i])
					txBuffer[bIdx][i] = AudioUSB_Base<StreamClass>::allocate();

				if(txBuffer[bIdx][i])
					memset(txBuffer[bIdx][i]->data, 0, AUDIO_BLOCK_SAMPLES*sizeof(txBuffer[bIdx][i]->data[0]));

				else {
					// we ran out of audio memory
					releaseBlocks(bIdx, noChannels);
					break;
				}
			}
		}
		_usbInterface.incrementBufferIndex();
	}

	void begin()
	{
		for (uint16_t i = 0; i < USBAudioOutInterface::ringTxBufferSize; i++)
			for (uint16_t j = 0; j < USB_AUDIO_MAX_NO_CHANNELS; j++)
				txBuffer[i][j] = NULL;
	}

	float getBufferedSamples() const
	{
		return _usbInterface.getBufferedSamples();
	}

	float getBufferedSamplesSmooth() const
	{
		return _usbInterface.getBufferedSamplesSmooth();
	}

	float getActualBIntervalUs() const
	{
		return _usbInterface.getActualBIntervalUs();
	}

	USBAudioOutInterface::Status getStatus() const
	{
		return _usbInterface.getStatus();
	}

private:

	static void copy_from_buffers(uint8_t *dst, uint16_t bIdx, uint16_t noChannels, unsigned int count, unsigned int len)
	{
		for (uint32_t i = 0; i < len; ++i) {
			for (uint16_t j = 0; j < noChannels; ++j) {
	#if USB_AUDIO_FORMAT == 1 // PCM
		#if AUDIO_SUBSLOT_SIZE>=2 && AUDIO_SUBSLOT_SIZE<=4
			#if OPENAUDIO
				union {
					int32_t i32;
					uint8_t u8[4];
				} tmp;
				constexpr auto scale = 1<<(sizeof(tmp.i32)*8-1);
				constexpr auto fmin = -1.f;
				constexpr auto fmax = float((scale-1.)/scale);
				// we need to clip incoming float data
				tmp.i32 = max(min(txBuffer[bIdx][j]->data[count+i], fmax), fmin)*float(scale);
				for(int k = 0; k < AUDIO_SUBSLOT_SIZE; ++k, ++dst)
					*dst = tmp.u8[k+(4-AUDIO_SUBSLOT_SIZE)];
			#else
				for(int k = 0; k < AUDIO_SUBSLOT_SIZE-2; ++k)
					*dst++ = 0; // zero low bytes
				int16_t* dst16Bit = (int16_t*)dst;
				*dst16Bit = txBuffer[bIdx][j]->data[count+i];
				dst += 2;
			#endif
		#else
			#error AUDIO_SUBSLOT_SIZE invalid
		#endif
	#elif USB_AUDIO_FORMAT == 4 // IEEE_FLOAT
			#if OPENAUDIO
				*dst = txBuffer[bIdx][j]->data[count+i];
			#else
				constexpr auto scale = 1<<(sizeof(int16_t)*8-1);
				*dst = txBuffer[bIdx][j]->data[count+i]*float32_t(1./scale);
			#endif
				dst += 4;
	#else
		#error USB_AUDIO_FORMAT invalid
	#endif
			}
		}
	}

	static void releaseBlocks(uint16_t bIdx, uint16_t noChannels) {  
		for (uint16_t i = 0; i < noChannels; i++)
			if(txBuffer[bIdx][i]) {
				AudioUSB_Base<StreamClass>::release(txBuffer[bIdx][i]);
				txBuffer[bIdx][i] = NULL;
			}
	}

	static bool isBlockReady(uint16_t bIdx, uint16_t channel) {
		return txBuffer[bIdx][channel] != NULL;
	}

	static typename AudioUSB_Base<StreamClass>::audio_block_t* txBuffer[USBAudioOutInterface::ringTxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];
	typename AudioUSB_Base<StreamClass>::audio_block_t *inputQueueArray[USB_AUDIO_MAX_NO_CHANNELS];
    USBAudioOutInterface _usbInterface;
};

template <typename StreamClass>
typename AudioUSB_Base<StreamClass>::audio_block_t* AudioOutputUSB_Proto<StreamClass>::txBuffer[USBAudioOutInterface::ringTxBufferSize][USB_AUDIO_MAX_NO_CHANNELS];


//////////////////////////////


using AudioInputUSB = AudioInputUSB_Proto<AudioStream_CLASS>;
using AudioOutputUSB = AudioOutputUSB_Proto<AudioStream_CLASS>;

#if USB_AUDIO_NO_CHANNELS_480 >= 4
class AudioInputUSBQuad: public AudioInputUSB { public: AudioInputUSBQuad(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
class AudioOutputUSBQuad: public AudioOutputUSB { public: AudioOutputUSBQuad(): AudioOutputUSB(4) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 6
class AudioInputUSBHex: public AudioInputUSB { public: AudioInputUSBHex(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
class AudioOutputUSBHex: public AudioOutputUSB { public: AudioOutputUSBHex(): AudioOutputUSB(6) {} };
#if USB_AUDIO_NO_CHANNELS_480 >= 8
class AudioInputUSBOct: public AudioInputUSB { public: AudioInputUSBOct(float kp =400.f,float ki =.2f) : AudioInputUSB(kp, ki) {} };
class AudioOutputUSBOct: public AudioOutputUSB { public: AudioOutputUSBOct(): AudioOutputUSB(8) {} };
#endif // USB_AUDIO_NO_CHANNELS_480 >= 8
#endif // USB_AUDIO_NO_CHANNELS_480 >= 6
#endif // USB_AUDIO_NO_CHANNELS_480 >= 4

#endif // AUDIO_INTERFACE
