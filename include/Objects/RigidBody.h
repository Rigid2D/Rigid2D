#ifndef RIGID2D_RIGID_BODY_H
#define RIGID2D_RIGID_BODY_H

#include "Common/RigidSettings.h"
#include "Common/MathUtils.h"
#include "Common/Vector2.h"
#include "Objects/Force.h"
#include "Collision/AABB.h"
#include <unordered_set>
#include <cmath>

namespace Rigid2D
{
  class Force;
  class RBSolver;

  // Stores the state needed for force calculations.
  struct RBState
  {
    enum FrameSpecifier { CURRENT, PREVIOUS };

    Vector2 position;
    Vector2 linearMomentum;
    Angle orientation;         // radians
    Real angularMomentum;

    RBState() {}

    RBState(const Vector2 &position, const Vector2 &linearMomentum,
        const Real &orientation, const Real &angularMomentum) :
      position(position),
      linearMomentum(linearMomentum),
      orientation(orientation),
      angularMomentum(angularMomentum) { }

    RBState(const RBState &other) :
        position(other.position),
        linearMomentum(other.linearMomentum),
        orientation(other.orientation),
        angularMomentum(other.angularMomentum) { }

    void operator *= (Real scalar);
    void operator /= (Real scalar);
    RBState operator + (const RBState & s) const;
    RBState operator - (const RBState & s) const;
    RBState operator * (const Real scalar) const;
    friend RBState operator * (const Real scalar, const RBState &state);
    RBState operator / (const Real scalar) const;
    void operator = (const RBState & other);
    void normalizeOrientAngle();
  };


  // Stores a rigid body - the base object used in the physics simulation.
  class RigidBody
  {
    public:
      /** Constructor for RigidBody
       * @param vertex_array should be an array of vertex coordinates given in counterclockwise order.
       *        Ex: vertex_array = {x0,y0,x1,y1,...,xn,yn}.
       *        The array is deep copied.
       * @param num_vertices is number of elements within vertex_array divided
       * by 2.  num_vertices must be equal to or greater than 3, otherwise
       * method will throw an InvalidParameterException.
       */
      RigidBody(unsigned int num_vertices,
                Real const *vertex_array,
                Vector2 const &position,
                Real mass,
                Vector2 const &velocity = Vector2(0,0),
                Angle orientation = 0.0,
                Real restitution = 1.0);

      // Same as above, but takes in a Vector2 array
      RigidBody(unsigned int num_vertices,
                Vector2 const *vertices,
                Vector2 const &position,
                Real mass,
                Vector2 const &velocity = Vector2(0,0),
                Angle orientation = 0.0,
                Real restitution = 1.0);

			RigidBody() {}

      ~RigidBody();

      /** Computes and sets the state of the object for next frame.
       * ODE computations are done from this function.
      */
      void update();

      /** Zeroes and then sums forces into forceAccumulator_ */
      void computeForces(RBState &state);
      void computeStateDeriv(const RBState &state, RBState &dState) const;

      bool checkCollision(RigidBody *rb);

      /** Tells RigidBodySystem to apply the given force from here on out.
			 * If the force was already previously given, it does not apply it a
			 * second time.
       *
       * @param force pointer to a Force object.
       * @see addForces()
       * @see removeForce()
       * @see removeForces()
       */
      void addForce(Force *force);

      /** Same as addForce() but for multiple forces.
			 *
       * @param forces array of Force object pointers.
			 * @param numForces length of forces.
       * @see addForce()
       * @see removeForce()
       * @see removeForces()
       */
			void addForces(Force **forces, unsigned int numForces);

      /** Tells RigidBodySystem to remove the force from the list of forces
			 * being applied. If the force was not being applied, it does nothing.
       *
       * @param force pointer to a Force object.
       * @see addForces()
       * @see addForces()
       * @see removeForces()
       */
			void removeForce(Force *force);

      /** Same as removeForce() but for multiple forces.
			 *
       * @param forces array of Force object pointers.
			 * @param numForces length of forces.
       * @see addForce()
       * @see addForces()
       * @see removeForce()
       */
			void removeForces (Force **forces, unsigned int numForces);

      Vector2 getPosition() const;
      Vector2 getVelocity() const;
      Vector2 getLinearMomentum() const;
      Real getAngularMomentum() const;
      Real getMass() const;
      Real getInvMass() const;
      Real getInvMoi() const;
      Angle getOrientation() const;
      Real getRestitution() const;

      /// Returns moment of inertia about axis through centroid and
      /// perpendicular to the plane of the RigidBody.
      Real getMomentOfInertia() const;

      void getState(RBState & state) const;
      void getPrevState(RBState & state) const;

			void setState(RBState & state);
			void setPrevState(RBState & state);
      void setPosition (const Vector2 & position);
      void setPosition (Real xPos, Real yPos);
      void setVelocity (const Vector2 & velocity);
      void setVelocity (Real xVel, Real yVel);
      void setOrientation (const Real orientation);
      void setMass (const Real mass);

      AABB* getStaticBB();
      AABB* getWorldBB();
      bool bp_isIntersecting() const;
      bool np_isIntersecting() const;

      /* Transform point in previous frame world space to current frame local space. */
      Vector2 prevWorldToCurrentLocalTransform(const Vector2 & point) const;

      // Transform point from world space to local body space using specified
      // state transform.  By default, FrameSpecifier is set to CURRENT so that
      // the current frame transform is used.  The FrameSpecifier can also be
      // assigned PREVIOUS in order to perfrom the coordinate transformation
      // using the transformation of the previous frame.
      Vector2 worldToLocalTransform(const Vector2 & point,
          RBState::FrameSpecifier frame = RBState::CURRENT) const;

      // Transform point from local body space to world space using specified
      // state transform.  By default, FrameSpecifier is set to CURRENT so that
      // the current frame transform is used.  The FrameSpecifier can also be
      // assigned PREVIOUS in order to perfrom the coordinate transformation
      // using the transformation of the previous frame.
      Vector2 localToWorldTransform(Vector2 const & point,
          RBState::FrameSpecifier frame = RBState::CURRENT) const;

      void updateTransformedVertices() const;

      unsigned int getNumVertices() const;
      Vector2 const * getVertices() const;

      // Returns a copy of the Vector2 object representing vertex at given
      // index of Rigid Body's vertex array.
      Vector2 getVertex(unsigned int index) const;

      /* Given a point in **world coordinate space**, this function returns true if
       * the point lies within the convex polygon defined by vertex_array_.*/
      bool pointIsInterior(Real x, Real y);

      // Returns the velocity of given point as if it were affixed to the Rigid
      // Body.  Takes into account both linear and angular velocities of the
      // Rigid Body during the current frame.  Assumes point is in local body
      // coordinates.
      Vector2 pointVelocity(Vector2 point);

    protected:
      /** Check for BB intersection. Call narrowPhase if true. */
      bool broadPhase(RigidBody *rb);

      /** Check for exact intersection. Called after broadPhase returns true. */
      bool narrowPhase(RigidBody *rb, bool firstRB = true);

    private:
      void initialize(unsigned int num_vertices,
                      Vector2 const *vertices,
                      Vector2 const &position,
                      Real mass,
                      Vector2 const &velocity,
                      Angle orientation,
                      Real restitution);

    Vector2 findProjectionInterval(const Vector2 & slope) const;

    protected:
      RBState state_;
      RBState prevState_;                     // The state last frame
      Vector2 velocity_;                      // Velocity of center of mass (implicitly calculated)
      Real mass_;                             // Mass of the Rigid Body
      Real invMass_;                          // 1/mass
      Real moi_;                              // Moment of inertia about axis perpendicular to plane of
                                              // body and through its center.
      Real invMoi_;                           // 1/moi
      Real restitution_;                      // Coefficient of restitution for collisions, in the interval [0,1].

      Vector2 forceAccumulator_;              // Sum of forces acting on the center of mass of RigidBody
      Real torqueAccumulator_;                // Sum of torques on body, about center of mass.

      std::unordered_set<Force*> forces_;     // All forces being applied to this RB

      // Geometry
      unsigned int num_vertices_;             // Number of vertices that make up the perimeter of RigidBody.
			Vector2 *vertices_;	                    // Collection of Vector2 objects representing the vertices that compose the RigidBody.
      Vector2 *transformed_vertices_;         // All the vertices in world space; updated on request
      AABB staticBB_;                         // local space, does not change
      AABB worldBB_;                          // world space, changes, used for broadPhase
      bool bp_isIntersecting_;                // is the body colliding in broadPhase
      bool np_isIntersecting_;                // is the body colliding in narrowPhase
  };
}

#endif
