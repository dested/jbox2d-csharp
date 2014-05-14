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
using org.jbox2d.common.Vec2;
using org.jbox2d.dynamics.Body;
using org.jbox2d.dynamics.SolverData;
using org.jbox2d.dynamics.World;
using org.jbox2d.pooling.IWorldPool;
// updated to rev 100
/**
 * The base joint class. Joints are used to constrain two bodies together in various fashions. Some
 * joints also feature limits and motors.
 * 
 * @author Daniel Murphy
 */
namespace org.jbox2d.dynamics.joints {
public abstract class Joint {

  public static Joint create(World world, JointDef def) {
    // Joint joint = null;
    switch (def.type) {
      case JointType.MOUSE:
        return new MouseJoint(world.getPool(), (MouseJointDef) def);
      case JointType.DISTANCE:
        return new DistanceJoint(world.getPool(), (DistanceJointDef) def);
      case JointType.PRISMATIC:
        return new PrismaticJoint(world.getPool(), (PrismaticJointDef) def);
      case JointType.REVOLUTE:
        return new RevoluteJoint(world.getPool(), (RevoluteJointDef) def);
      case JointType.WELD:
        return new WeldJoint(world.getPool(), (WeldJointDef) def);
      case JointType.FRICTION:
        return new FrictionJoint(world.getPool(), (FrictionJointDef) def);
      case JointType.WHEEL:
        return new WheelJoint(world.getPool(), (WheelJointDef) def);
      case JointType.GEAR:
        return new GearJoint(world.getPool(), (GearJointDef) def);
      case JointType.PULLEY:
        return new PulleyJoint(world.getPool(), (PulleyJointDef) def);
      case JointType.CONSTANT_VOLUME:
        return new ConstantVolumeJoint(world, (ConstantVolumeJointDef) def);
      case JointType.ROPE:
        return new RopeJoint(world.getPool(), (RopeJointDef) def);
      case JointType.UNKNOWN:
      default:
        return null;
    }
  }

  public static void destroy(Joint joint) {
    joint.destructor();
  }

  private readonly JointType m_type;
  public Joint m_prev;
  public Joint m_next;
  public JointEdge m_edgeA;
  public JointEdge m_edgeB;
  public Body m_bodyA;
  public Body m_bodyB;

  public bool m_islandFlag;
  private bool m_collideConnected;

  public object m_userData;

  public IWorldPool pool;

  // Cache here per time step to reduce cache misses.
  // Vec2 m_localCenterA, m_localCenterB;
  // float m_invMassA, m_invIA;
  // float m_invMassB, m_invIB;

  public Joint(IWorldPool worldPool, JointDef def) {
    assert (def.bodyA != def.bodyB);

    pool = worldPool;
    m_type = def.type;
    m_prev = null;
    m_next = null;
    m_bodyA = def.bodyA;
    m_bodyB = def.bodyB;
    m_collideConnected = def.collideConnected;
    m_islandFlag = false;
    m_userData = def.userData;

    m_edgeA = new JointEdge();
    m_edgeA.joint = null;
    m_edgeA.other = null;
    m_edgeA.prev = null;
    m_edgeA.next = null;

    m_edgeB = new JointEdge();
    m_edgeB.joint = null;
    m_edgeB.other = null;
    m_edgeB.prev = null;
    m_edgeB.next = null;

    // m_localCenterA = new Vec2();
    // m_localCenterB = new Vec2();
  }

  /**
   * get the type of the concrete joint.
   * 
   * @return
   */
  public JointType getType() {
    return m_type;
  }

  /**
   * get the first body attached to this joint.
   */
  public  Body getBodyA() {
    return m_bodyA;
  }

  /**
   * get the second body attached to this joint.
   * 
   * @return
   */
  public  Body getBodyB() {
    return m_bodyB;
  }

  /**
   * get the anchor point on bodyA in world coordinates.
   * 
   * @return
   */
  public abstract void getAnchorA(Vec2 out_);

  /**
   * get the anchor point on bodyB in world coordinates.
   * 
   * @return
   */
  public abstract void getAnchorB(Vec2 out_);

  /**
   * get the reaction force on body2 at the joint anchor in Newtons.
   * 
   * @param inv_dt
   * @return
   */
  public abstract void getReactionForce(float inv_dt, Vec2 out_);

  /**
   * get the reaction torque on body2 in N*m.
   * 
   * @param inv_dt
   * @return
   */
  public abstract float getReactionTorque(float inv_dt);

  /**
   * get the next joint the world joint list.
   */
  public Joint getNext() {
    return m_next;
  }

  /**
   * get the user data pointer.
   */
  public object getUserData() {
    return m_userData;
  }

  /**
   * Set the user data pointer.
   */
  public void setUserData(object data) {
    m_userData = data;
  }

  // / Get collide connected.
  // / Note: modifying the collide connect flag won't work correctly because
  // / the flag is only checked when fixture AABBs begin to overlap.
  public  bool getCollideConnected() {
    return m_collideConnected;
  }

  /**
   * Short-cut function to determine if either body is inactive.
   * 
   * @return
   */
  public bool isActive() {
    return m_bodyA.isActive() && m_bodyB.isActive();
  }

  public abstract void initVelocityConstraints(SolverData data);

  public abstract void solveVelocityConstraints(SolverData data);

  /**
   * This returns true if the position errors are within tolerance.
   * 
   * @param baumgarte
   * @return
   */
  public abstract bool solvePositionConstraints(SolverData data);

  /**
   * Override to handle destruction of joint
   */
  public void destructor() {}
}
}