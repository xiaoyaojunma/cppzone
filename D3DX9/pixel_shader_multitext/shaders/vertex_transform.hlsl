/*
 *  Copyright (c) 2012 Evgeny Proydakov <lord.tiran@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

matrix ViewProjMatrix;

struct VS_INPUT
{
     vector position  : POSITION0;

     float2 tex0coord : TEXCOORD0;
     float2 tex1coord : TEXCOORD1;
     float2 tex2coord : TEXCOORD2;
};

struct VS_OUTPUT
{
     vector position  : POSITION;

     float2 tex0coord : TEXCOORD0;
     float2 tex1coord : TEXCOORD1;
     float2 tex2coord : TEXCOORD2;
};

VS_OUTPUT main(VS_INPUT input)
{
     VS_OUTPUT output = (VS_OUTPUT)0;
     output.position = mul(input.position, ViewProjMatrix);

     output.tex0coord = input.tex0coord;
     output.tex1coord = input.tex1coord;
     output.tex2coord = input.tex2coord;

     return output;
}