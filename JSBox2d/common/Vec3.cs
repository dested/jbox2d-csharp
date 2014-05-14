/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY out_ OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


using System;
using System.IO;

/**
 * @author Daniel Murphy
 */
namespace org.jbox2d.common {
public class Vec3 {
  private static readonly long serialVersionUID = 1L;

  public float x, y, z;

  public Vec3() {
    x = y = z = 0f;
  }

  public Vec3(float argX, float argY, float argZ) {
    x = argX;
    y = argY;
    z = argZ;
  }

  public Vec3(Vec3 argCopy) {
    x = argCopy.x;
    y = argCopy.y;
    z = argCopy.z;
  }

  public Vec3 set(Vec3 argVec) {
    x = argVec.x;
    y = argVec.y;
    z = argVec.z;
    return this;
  }

  public Vec3 set(float argX, float argY, float argZ) {
    x = argX;
    y = argY;
    z = argZ;
    return this;
  }

  public Vec3 addLocal(Vec3 argVec) {
    x += argVec.x;
    y += argVec.y;
    z += argVec.z;
    return this;
  }

  public Vec3 add(Vec3 argVec) {
    return new Vec3(x + argVec.x, y + argVec.y, z + argVec.z);
  }

  public Vec3 subLocal(Vec3 argVec) {
    x -= argVec.x;
    y -= argVec.y;
    z -= argVec.z;
    return this;
  }

  public Vec3 sub(Vec3 argVec) {
    return new Vec3(x - argVec.x, y - argVec.y, z - argVec.z);
  }

  public Vec3 mulLocal(float argScalar) {
    x *= argScalar;
    y *= argScalar;
    z *= argScalar;
    return this;
  }

  public Vec3 mul(float argScalar) {
    return new Vec3(x * argScalar, y * argScalar, z * argScalar);
  }

  public Vec3 negate() {
    return new Vec3(-x, -y, -z);
  }

  public Vec3 negateLocal() {
    x = -x;
    y = -y;
    z = -z;
    return this;
  }

  public void setZero() {
    x = 0;
    y = 0;
    z = 0;
  }

  public Vec3 clone() {
    return new Vec3(this);
  }

  public string toString() {
    return "(" + x + "," + y + "," + z + ")";
  }

  
  public override int GetHashCode() {
     int prime = 31;
    int result = 1;
    result = prime * result + NumberHelper.floatToIntBits(x);
    result = prime * result + NumberHelper.floatToIntBits(y);
    result = prime * result + NumberHelper.floatToIntBits(z);
    return result;
  }

  
  public override bool Equals(object obj) {
    if (this == obj) return true;
    if (obj == null) return false;
    if (GetType() != obj.GetType()) return false;
    Vec3 other = (Vec3) obj;
    if (NumberHelper.floatToIntBits(x) != NumberHelper.floatToIntBits(other.x)) return false;
    if (NumberHelper.floatToIntBits(y) != NumberHelper.floatToIntBits(other.y)) return false;
    if (NumberHelper.floatToIntBits(z) != NumberHelper.floatToIntBits(other.z)) return false;
    return true;
  }

  public  static float dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  public  static Vec3 cross(Vec3 a, Vec3 b) {
    return new Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  }

  public  static void crossToOut(Vec3 a, Vec3 b, Vec3 out_) {
     float tempy = a.z * b.x - a.x * b.z;
     float tempz = a.x * b.y - a.y * b.x;
    out_.x = a.y * b.z - a.z * b.y;
    out_.y = tempy;
    out_.z = tempz;
  }
  
  public  static void crossToOutUnsafe(Vec3 a, Vec3 b, Vec3 out_) {
    out_.x = a.y * b.z - a.z * b.y;
    out_.y = a.z * b.x - a.x * b.z;
    out_.z = a.x * b.y - a.y * b.x;
  }
}

    public static class NumberHelper
    {
        public static int floatToIntBits(float j)
        {
            return BitConverter.ToInt32(BitConverter.GetBytes(j), 0);
        }

        public static float floatToRawIntBits(float f)
        {
            byte[] bytes = BitConverter.GetBytes(f);

            int result = BitConverter.ToInt32(bytes, 0);
            return result;
        }

        public static float intBitsToFloat(int x)
        {
            byte[] bytes = BitConverter.GetBytes(x);
            float f = BitConverter.ToSingle(bytes, 0);
            return f;
        }
    }
}