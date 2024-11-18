#ifndef FINGERPOSITION_H
#define FINGERPOSITION_H
namespace Gauntl33t
{
  /**
   * Define PI2 for convenience.
   */
  namespace MATH
  {
  const float PI2 = PI * 2.0;
  }

  /**
   * Class for tracking finger position.
   */
  class FingerPosition
  {
    private:
    float mMaxAngle_Cumulative;  //Keeps track of max cumulative angle recorded.
    float mMinAngle_Cumulative;  //Keeps track of min cumulative angle recorded.
    float mCurAngle_Cumulative;  //Keeps track of the cumulative current angle.
    float mPrevAngle_Cumulative; //Stores the previous cumulative angle.
    float mPrevAngle; //Stores the raw previous angle
    public:
    /**
     * Constructor.
     * Initialize finger position with starting sensor angle.
     */
    FingerPosition(float aStartAngle);
    /**
     * Call every frame with new angle so that the position is acturate.
     * @param aAngle read from sensor to be used in updating finger position.
     */
    void SensorUpdatePos(float aAngle);
    /**
     * Returns normalized value between 0.0 and 1.0 of finger position.
     */
    float GetFingerPosition();
  };
}
#endif
