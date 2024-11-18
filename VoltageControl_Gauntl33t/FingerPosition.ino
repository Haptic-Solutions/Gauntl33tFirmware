namespace Gauntl33t
{
  /**
   * Constructor.
   * Initialize finger position with starting sensor angle.
   * Set initializer list variables to aStartAngle.
   * @param aStartAngle to initialize finger position with.
   */
  FingerPosition::FingerPosition(float aStartAngle):
  mMaxAngle_Cumulative(aStartAngle),
  mMinAngle_Cumulative(aStartAngle),
  mCurAngle_Cumulative(aStartAngle),
  mPrevAngle_Cumulative(aStartAngle),
  mPrevAngle(aStartAngle)
  {
  }
  /**
   * Call every frame with new angle so that the position is acturate.
   * @param aAngle read from sensor to be used in updating finger position.
   */
  void FingerPosition::SensorUpdatePos(float aAngle)
  {
    //Get non cumulative dif between sensor angle and the previous raw sensor angle.
    float dif = abs(aAngle - mPrevAngle);
    //If there has been no change in position just return.
    if(dif == 0.0f)
    {
      return;
    }
    //If the non cumulative angle is greater than the previous non cumulative angle.
    if(aAngle > mPrevAngle)
    {
      //If the difference is less than half a rotation add the difference to the cumulative total.
      if(dif < PI)
      {
        mCurAngle_Cumulative += dif;
      }
      //Otherwise presume passed the boundry and compute a decrease. Subtract from cumulative angle.
      else
      {
        mCurAngle_Cumulative -= ((mPrevAngle - 0.0f) + (MATH::PI2 - aAngle));
      }
    }
    else
    {
      //If the difference is less than half a rotation subtract the difference from the cumulative total.
      if(dif < PI)
      {
        mCurAngle_Cumulative -= dif;
      }
      //Otherwise presume passed the boundary and compute increase. Add to cumulative angle.
      else
      {
        mCurAngle_Cumulative += ((MATH::PI2 - mPrevAngle) + aAngle);
      }
    }
    //Check if new cumulative max and store.
    if(mMaxAngle_Cumulative < mCurAngle_Cumulative)
    {
      mMaxAngle_Cumulative = mCurAngle_Cumulative;
      //Serial.print(String(mMaxAngle_Cumulative).c_str());
    }
    //Check if new min.
    if(mMinAngle_Cumulative > mCurAngle_Cumulative)
    {
      mMinAngle_Cumulative = mCurAngle_Cumulative;
      //Serial.print(String(mMinAngle_Cumulative).c_str());
    }
    //Update prev angle
    mPrevAngle = aAngle;
    mPrevAngle_Cumulative = mCurAngle_Cumulative;
  }
  /**
   * Returns normalized value between 0.0 and 1.0 of finger position.
   */
  float FingerPosition::GetFingerPosition()
  {
    float dif = mMaxAngle_Cumulative - mMinAngle_Cumulative;
    //Serial.write(String(dif).c_str());
    float val = mCurAngle_Cumulative + (0.0f - mMinAngle_Cumulative);
    //Avoid divide by 0 error.
    if(dif == 0.0f)
    {
      return 0.0;
    }
    return val/dif;
  }
}
