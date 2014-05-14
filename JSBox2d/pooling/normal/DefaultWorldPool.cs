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
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 3:26:14 AM Jan 11, 2011
 */


using System;
using org.jbox2d.collision.AABB;
using org.jbox2d.collision.Collision;
using org.jbox2d.collision.Distance;
using org.jbox2d.collision.TimeOfImpact;
using org.jbox2d.common.Mat22;
using org.jbox2d.common.Mat33;
using org.jbox2d.common.Rot;
using org.jbox2d.common.Settings;
using org.jbox2d.common.Vec2;
using org.jbox2d.common.Vec3;
using org.jbox2d.dynamics.contacts.ChainAndCircleContact;
using org.jbox2d.dynamics.contacts.ChainAndPolygonContact;
using org.jbox2d.dynamics.contacts.CircleContact;
using org.jbox2d.dynamics.contacts.Contact;
using org.jbox2d.dynamics.contacts.EdgeAndCircleContact;
using org.jbox2d.dynamics.contacts.EdgeAndPolygonContact;
using org.jbox2d.dynamics.contacts.PolygonAndCircleContact;
using org.jbox2d.dynamics.contacts.PolygonContact;
using org.jbox2d.pooling.IDynamicStack;
using org.jbox2d.pooling.IWorldPool;
using System.Collections.Generic;
using System.Collections;
/**
 * Provides object pooling for all objects used in the engine. Objects retrieved from here should
 * only be used temporarily, and then pushed back (with the exception of arrays).
 * 
 * @author Daniel Murphy
 */
namespace org.jbox2d.pooling.normal {
public class DefaultWorldPool : IWorldPool {

  private readonly OrderedStack<Vec2> vecs;
  private readonly OrderedStack<Vec3> vec3s;
  private readonly OrderedStack<Mat22> mats;
  private readonly OrderedStack<Mat33> mat33s;
  private readonly OrderedStack<AABB> aabbs;
  private readonly OrderedStack<Rot> rots;

  private readonly Hashtable<int?, float[]> afloats = new Hashtable<int?, float[]>();
  private readonly Hashtable<int?, int[]> aints = new Hashtable<int?, int[]>();
  private readonly Hashtable<int?, Vec2[]> avecs = new Hashtable<int?, Vec2[]>();

  private readonly IWorldPool world = this;
/*

  private readonly MutableStack<Contact> pcstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new PolygonContact(world); }
    };

  private readonly MutableStack<Contact> ccstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new CircleContact(world); }
    };

  private readonly MutableStack<Contact> cpstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new PolygonAndCircleContact(world); }
    };

  private readonly MutableStack<Contact> ecstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new EdgeAndCircleContact(world); }
    };

  private readonly MutableStack<Contact> epstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new EdgeAndPolygonContact(world); }
    };

  private readonly MutableStack<Contact> chcstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new ChainAndCircleContact(world); }
    };

  private readonly MutableStack<Contact> chpstack =
    new MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
      public Contact newInstance () { return new ChainAndPolygonContact(world); }
    };

  private readonly Collision collision;
  private readonly TimeOfImpact toi;
  private readonly Distance dist;

  public DefaultWorldPool(int argSize, int argContainerSize) {
    vecs = new OrderedStack<Vec2>(argSize, argContainerSize) {
      public Vec2 newInstance() { return new Vec2(); }
    };
    vec3s = new OrderedStack<Vec3>(argSize, argContainerSize) {
      public Vec3 newInstance() { return new Vec3(); }
    };
    mats = new OrderedStack<Mat22>(argSize, argContainerSize) {
      public Mat22 newInstance() { return new Mat22(); }
    };
    aabbs = new OrderedStack<AABB>(argSize, argContainerSize) {
      public AABB newInstance() { return new AABB(); }
    };
    rots = new OrderedStack<Rot>(argSize, argContainerSize) {
      public Rot newInstance() { return new Rot(); }
    };
    mat33s = new OrderedStack<Mat33>(argSize, argContainerSize) {
      public Mat33 newInstance() { return new Mat33(); }
    };

    dist = new Distance();
    collision = new Collision(this);
    toi = new TimeOfImpact(this);
  }
*/

  public  IDynamicStack<Contact> getPolyContactStack() {
    return pcstack;
  }

  public  IDynamicStack<Contact> getCircleContactStack() {
    return ccstack;
  }

  public  IDynamicStack<Contact> getPolyCircleContactStack() {
    return cpstack;
  }

  
  public override IDynamicStack<Contact> getEdgeCircleContactStack() {
    return ecstack;
  }

  
  public override IDynamicStack<Contact> getEdgePolyContactStack() {
    return epstack;
  }

  
  public override IDynamicStack<Contact> getChainCircleContactStack() {
    return chcstack;
  }

  
  public override IDynamicStack<Contact> getChainPolyContactStack() {
    return chpstack;
  }

  public  Vec2 popVec2() {
    return vecs.pop();
  }

  public  Vec2[] popVec2(int argNum) {
    return vecs.pop(argNum);
  }

  public  void pushVec2(int argNum) {
    vecs.push(argNum);
  }

  public  Vec3 popVec3() {
    return vec3s.pop();
  }

  public  Vec3[] popVec3(int argNum) {
    return vec3s.pop(argNum);
  }

  public  void pushVec3(int argNum) {
    vec3s.push(argNum);
  }

  public  Mat22 popMat22() {
    return mats.pop();
  }

  public  Mat22[] popMat22(int argNum) {
    return mats.pop(argNum);
  }

  public  void pushMat22(int argNum) {
    mats.push(argNum);
  }

  public  Mat33 popMat33() {
    return mat33s.pop();
  }

  public  void pushMat33(int argNum) {
    mat33s.push(argNum);
  }

  public  AABB popAABB() {
    return aabbs.pop();
  }

  public  AABB[] popAABB(int argNum) {
    return aabbs.pop(argNum);
  }

  public  void pushAABB(int argNum) {
    aabbs.push(argNum);
  }

  public  Rot popRot() {
    return rots.pop();
  }

  public  void pushRot(int num) {
    rots.push(num);
  }

  public  Collision getCollision() {
    return collision;
  }

  public  TimeOfImpact getTimeOfImpact() {
    return toi;
  }

  public  Distance getDistance() {
    return dist;
  }

  public  float[] getFloatArray(int argLength) {
    if (!afloats.containsKey(argLength)) {
      afloats.put(argLength, new float[argLength]);
    }

    return afloats.get(argLength);
  }

  public  int[] getIntArray(int argLength) {
    if (!aints.containsKey(argLength)) {
      aints.put(argLength, new int[argLength]);
    }

    return aints.get(argLength);
  }

  public  Vec2[] getVec2Array(int argLength) {
    if (!avecs.containsKey(argLength)) {
      Vec2[] ray = new Vec2[argLength];
      for (int i = 0; i < argLength; i++) {
        ray[i] = new Vec2();
      }
      avecs.put(argLength, ray);
    }

    return avecs.get(argLength);
  }
}
}