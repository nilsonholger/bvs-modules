////////////////////////////////////////////////////////////////////////
// micro JPEG encoder
//
// Manuel Martinez (salutte@gmail.com)
// based on: jpge from Rich Geldreich
//
// Stripped down JPEG encoder with the base essentials. Compression is
// fixed to baseline YUV 411. Suit yourself to modify it for your own
// needs.
// 
// The goal of this library is to provide a small but clear baseline jpeg
// implementation for teaching and experimentation purposes.
//
// license: LGPLv3

#pragma once

#include <string>
#include <cstdio>
#include <cmath>
#include <vector>
#include <cassert>

class uJpge {

	typedef unsigned char byte;

	////////////////////////////////////////////////////////////////////
	// DCT Class

	// Templated generic DCT transform class. Not meant to be efficient
	// Complexity O(n^3) instead of O(n^2 log n)
	template<int N, typename T>
	class DCT {
	public:
		
		T U[N*N];
		
		DCT() {
			
			for (int i=0; i<N*N; i++)
				U[i] = M(i/N,i%N);

		};
		
		static T M(int n, int k) {

			static const long double pi = acos(-1.L);
			long double r = sqrt(.25L)*8.L/N;
			if (n==0) r = sqrt(.128L)*8.L/N;
			return T(r*cos(n*pi*(2*k+1)/(2*N)));
		}
		
		// 1D transform is the naive matrix multiplication
		template<typename Q>
		void D1D(Q *out, const Q *in, int sk=1) {
			
			for (int i=0; i<N; i++) {
				out[i*sk] = 0;
				for (int j=0; j<N; j++)
					out[i*sk] += in[j*sk]*U[N*i+j];
			}
		}

		// 2D transform is performed using row/colum decomposition
		template<typename Q>
		void D2D(Q *out, const Q *in) {

			Q tmp[N*N];
			
			for (int i=0; i<N; i++)
				D1D(tmp+i*N, in+i*N); 

			for (int i=0; i<N; i++)
				D1D(out+i, tmp+i, N); 
		}
	};

	DCT<8,double> dct;

	////////////////////////////////////////////////////////////////////
	// STREAMING

	class Stream {
		std::string out;
		unsigned int outb;
		int outsz;
		
	public:
		Stream() {
			
			out.clear();
			outsz = 0;
			outb = 0;
		}
		
		void push(int b) {
			
			push(0x7F, 7); outsz = 0; outb = 0; //Align to byte;
			out.push_back(b);
		}

		void pushW(int b) {
			
			push(0x7F, 7); outsz = 0; outb = 0; //Align to byte;
			out.push_back(char(b/256));
			out.push_back(char(b%256));
		}
		
		void push(int val, int len) {

			val = (val & ((1<<len)-1)); //Masking out bits
			
			outsz += len;
			outb |= (val << (24 - outsz));
			while (outsz >= 8) {
				
				byte c = byte((outb >> 16) & 0xFF);
				out.push_back(c);
				if (c == 0xFF) out.push_back(0); //Byte stuffing

				outb <<= 8;
				outsz -= 8;
			}
		}
		
		operator std::string() { return out; }
	};
	
	Stream stream;
	
	////////////////////////////////////////////////////////////////////
	// BGR -> YCC

	static void BGR2YCC(byte* img, int sz) {
		
		//Slightly altered values to avoid clamming
		const int YR = 19595, YG = 38470, YB = 7471, CB_R = -11059, CB_G = -21709, CB_B = 32767, CR_R = 32767, CR_G = -27439, CR_B = -5329;
		for (int i=0; i<sz; i++) {
			const int b = img[0], g = img[1], r = img[2];
			*img++ = (r * YR + g * YG + b * YB + 32767) >> 16;
			*img++ = 128 + ((r * CB_R + g * CB_G + b * CB_B + 32768) >> 16);
			*img++ = 128 + ((r * CR_R + g * CR_G + b * CR_B + 32768) >> 16);
		}
	}

	////////////////////////////////////////////////////////////////////
	// HEADER GENERATION SECTION
	
	void soi() {
		stream.pushW(0xFFD8); //SOS
	}

	void eoi() {
		stream.pushW(0xFFD9); //EOI
	}

	// Emit quantization tables
	void dqt(const int *QY, const int *QC) {
		
		stream.pushW(0xFFDB); //DQT
		stream.pushW(64 + 1 + 2); //Size
		stream.push(0); // N Table
		for (int j = 0; j < 64; j++)
			stream.push(QY[j]);

		stream.pushW(0xFFDB); //DQT
		stream.pushW(64 + 1 + 2); //Size
		stream.push(1); // N Table
		for (int j = 0; j < 64; j++)
			stream.push(QC[j]);
	}

	// Emit start of frame marker
	void sof(int width, int height) {

		stream.pushW(0xFFC0); //SOF
		stream.pushW(3 * 3 + 2 + 5 + 1); //Size
		stream.push(8); //Precision
		stream.pushW(height);
		stream.pushW(width);
		stream.push(3); //N components

		stream.push(1); //Comp ID
		stream.push(0x22); //Comp sampling
		stream.push(0); //Q table ID

		stream.push(2); //Comp ID
		stream.push(0x11); //Comp sampling
		stream.push(1); //Q table ID

		stream.push(3); //Comp ID
		stream.push(0x11); //Comp sampling
		stream.push(1); //Q table ID
	}

	// Emit Huffman table.
	void dht(const int *HV, const int *HS, int index, bool ac) {

		stream.pushW(0xFFC4); //DHT

		int sz = 0;
		for (int i = 1; i <= 16; i++)
			sz += HS[i];
			
		stream.pushW(2 + 1 + 16 + sz);
		if (ac) stream.push(0x10+index);
		else stream.push(index);

		for (int i = 1; i <= 16; i++)
			stream.push(HS[i]);

		for (int i = 0; i < sz; i++)
			stream.push(HV[i]);
	}

	// emit start of scan
	void sos() {
		
		stream.pushW(0xFFDA); //SOS
		stream.pushW(2 * 3 + 2 + 1 + 3);
		stream.push(3); // N components
		stream.push(1); stream.push(0x00); //Comp ID + huff tables
		stream.push(2); stream.push(0x11); //Comp ID + huff tables
		stream.push(3); stream.push(0x11); //Comp ID + huff tables
		stream.push(0);  // Start of spectral selection
		stream.push(63); // End of spectral selection
		stream.push(0);  // Successive approximation
	}


	////////////////////////////////////////////////////////////////////
	// TABLE PREPROCESSING

	// Quantization table generation.
	void getQuantizationTable(int *Q, const int *SQ, int quality) {
		
	  int q = 5000 / quality;
	  if (quality > 50)
		q = 200 - quality * 2;
		
	  for (int i = 0; i < 64; i++) 
		Q[i] = std::min(std::max((SQ[i]*q+49)/100, 1), 255);
	}

	// Compute the actual canonical Huffman codes/code sizes given the JPEG representation
	void getCanonicalHuffman(int *CHV, int *CHS, const int *HV, const int *HS) {
		
		int code = 0, n = 0;
		for (int l = 1; l <= 16; l++) {
			for (int i = 1; i <= HS[l]; i++) {
				CHV[HV[n]] = code++;
				CHS[HV[n]] = l;
				n++;
			}
			code <<=1;
		}
	}

	////////////////////////////////////////////////////////////////////
	// ACTUAL COMPRESSION

	void block(byte *p, int sx, int sy, const int *Q, int &lastDC, const int *CHDV, const int *CHDS, const int *CHAV, const int *CHAS) {
		
		// ZIGZAG CODE
		static const int ZZ[]   = { 0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63 };

		// Load block
		double block[64];
		for (int i=0; i<8; i++)
			for (int j=0; j<8; j++)
				block[i*8+j] = p[3*sy*i+3*sx*j]-128;

		// DCT
		double blockDCT[64];
		dct.D2D(blockDCT, block);
		
		
		// Quantization && Zig Zag
		int blockQ[64];
		for (int i=0; i<64; i++)
			blockQ[i] = round(blockDCT[ZZ[i]]/Q[i]);

		// Store DC
		int dc = blockQ[0] - lastDC;
		lastDC = blockQ[0];
		
		int dcs = 0;
		while ((1<<dcs) <= std::abs(dc)) dcs++;

		stream.push(CHDV[dcs], CHDS[dcs]);
		if (dcs)
			stream.push(dc-(dc<0),dcs);
		
		//push(CHAV[0], CHAS[0]); return; //process only DC coefficients
		
		// Store AC
		int rle = 0;
		for (int i=1; i<64; i++) {
			if (blockQ[i]==0) {

				rle++;
			} else {
				
				while (rle>=16) { // Special case of long rle
					stream.push(CHAV[0xF0], CHAS[0xF0]);
					rle -= 16;
				}
				
				int acs = 0;
				while ((1<<acs) <= std::abs(blockQ[i])) acs++;
				
				int code = (rle<<4)+acs;
				stream.push(CHAV[code],CHAS[code]);
				stream.push(blockQ[i]-(blockQ[i]<0),acs);
				rle = 0;
			}
		}
		if (rle) 
			stream.push(CHAV[0], CHAS[0]); //Send EOB
	}
	
	////////////////////////////////////////////////////////////////////
	// PRIVATE ENCODING FUNCTION
	
	std::string pencode(byte *src, int width, int height, int quality) {

		// Q Tables [Y-lum|C-chroma]
		static const int SQY[]  = { 16,11,12,14,12,10,16,14,13,14,18,17,16,19,24,40,26,24,22,22,24,49,35,37,29,40,58,51,61,60,57,51,56,55,64,72,92,78,64,68,87,69,55,56,80,109,81,87,95,98,103,104,103,62,77,113,121,112,100,120,92,101,103,99 };
		static const int SQC[]  = { 17,18,18,24,21,24,47,26,26,47,99,66,56,66,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99 };
		// Huff Tables [Y-lum|C-chroma][D-dc|A-ac][S-size|V-values]
		static const int HYDS[] = { 0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 };
		static const int HYDV[] = { 0,1,2,3,4,5,6,7,8,9,10,11 };
		static const int HYAS[] = { 0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d };
		static const int HYAV[] = { 0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0, 0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49, 0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89, 0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5, 0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8, 0xf9,0xfa };
		static const int HCDS[] = { 0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0 };
		static const int HCDV[] = { 0,1,2,3,4,5,6,7,8,9,10,11 };
		static const int HCAS[] = { 0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77 };
		static const int HCAV[] = { 0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0, 0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48, 0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87, 0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3, 0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa };
		
		assert( width%16 == 0 ); // Only images with size multiple of 16 are supported
		assert( height%16 == 0 );
		assert( quality>0 and quality<=100 );
		
		std::vector<byte> img(&src[0], &src[width*height*3]);
		BGR2YCC(&img[0], width*height);
		
		int QY[64], QC[64];
		getQuantizationTable(QY, SQY, quality);
		getQuantizationTable(QC, SQC, quality);
		
		int CHYDV[256], CHYDS[256];
		int CHYAV[256], CHYAS[256];
		int CHCDV[256], CHCDS[256];
		int CHCAV[256], CHCAS[256];
		getCanonicalHuffman(CHYDV, CHYDS, HYDV, HYDS);
		getCanonicalHuffman(CHYAV, CHYAS, HYAV, HYAS);
		getCanonicalHuffman(CHCDV, CHCDS, HCDV, HCDS);
		getCanonicalHuffman(CHCAV, CHCAS, HCAV, HCAS);

		// Start writing the image
		stream = Stream();
		soi();
		dqt(QY, QC);
		sof(width, height);
		dht(HYDV, HYDS, 0, false);
		dht(HYAV, HYAS, 0, true);
		dht(HCDV, HCDS, 1, false);
		dht(HCAV, HCAS, 1, true);

		sos();	
		// Code image
		int lastDCY = 0, lastDCU = 0, lastDCV = 0;
		for (int i=0; i<height; i+=16) {
			for (int j=0; j<width; j+=16) {
				block(&img[(i+0)*width*3 + (j+0)*3 + 0], 1,   width, QY, lastDCY, CHYDV, CHYDS, CHYAV, CHYAS); 
				block(&img[(i+0)*width*3 + (j+8)*3 + 0], 1,   width, QY, lastDCY, CHYDV, CHYDS, CHYAV, CHYAS); 
				block(&img[(i+8)*width*3 + (j+0)*3 + 0], 1,   width, QY, lastDCY, CHYDV, CHYDS, CHYAV, CHYAS); 
				block(&img[(i+8)*width*3 + (j+8)*3 + 0], 1,   width, QY, lastDCY, CHYDV, CHYDS, CHYAV, CHYAS);  
				block(&img[(i+0)*width*3 + (j+0)*3 + 1], 2, 2*width, QC, lastDCU, CHCDV, CHCDS, CHCAV, CHCAS);  
				block(&img[(i+0)*width*3 + (j+0)*3 + 2], 2, 2*width, QC, lastDCV, CHCDV, CHCDS, CHCAV, CHCAS);  
			}
		}
		
		// Emit Footer
		eoi();
		
		return stream;
	}
	
public:

	static std::string encode(byte *src, int width, int height, int quality = 50) {
		
		return uJpge().pencode(src, width, height, quality);
	}
};

// Thanks for R.T.F.C.
