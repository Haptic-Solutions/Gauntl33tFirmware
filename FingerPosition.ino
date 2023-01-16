namespace Gauntl33t
{
  FingerPosition::FingerPosition(float aStartAngle):
  mMaxAngle_Cumulative(aStartAngle),
  mMinAngle_Cumulative(aStartAngle),
  mCurAngle_Cumulative(aStartAngle),
  mPrevAngle_Cumulative(aStartAngle),
  mPrevAngle(aStartAngle)
  {
  }
  void FingerPosition::SensorUpdatePos(float aAngle)
  {
    //get non cumulative dif
    auto dif = abs(aAngle - mPrevAngle);
    //if the non cumulative angle is greater than the previous non cumulative angle.
    if(aAngle > mPrevAngle)
    {
      //if the difference is less than half a rotation add the difference to the cumulative total.
      if(dif < PI)
      {
        mCurAngle_Cumulative += dif;
      }
      //otherwise presume passed the boundry and compute a decrease. Subtract from cumulative angle.
      else
      {
        mCurAngle_Cumulative -= ((mPrevAngle - 0.0f) + (MATH::PI2 - aAngle));
      }
    }
    else
    {
      //if the difference is less than half a rotation subtract the difference from the cumulative total.
      if(dif < PI)
      {
        mMinAngle_Cumulative -= dif;
      }
      //otherwise presume passed the boundary and compute increase. Add to cumulative angle.
      else
      {
        mCurAngle_Cumulative += ((MATH::PI2 - mPrevAngle) + aAngle);
      }
    }
    //check if new cumulative max and store
    if(mMaxAngle_Cumulative < mCurAngle_Cumulative)
    {
      mMaxAngle_Cumulative = mCurAngle_Cumulative;
    }
    //check if new min
    if(mMinAngle_Cumulative > mCurAngle_Cumulative)
    {
      mMinAngle_Cumulative = mCurAngle_Cumulative;
    }
    //update prev angle
    mPrevAngle = aAngle;
    mPrevAngle_Cumulative = mCurAngle_Cumulative;
  }
  //send normalized value between 0.0 and 1.0
  float FingerPosition::GetFingerPosition()
  {
    auto dif = mMaxAngle_Cumulative - mMinAngle_Cumulative;
    auto val = mCurAngle_Cumulative;
    if(mMinAngle_Cumulative < 0.0f)
    {
      val += (0.0f - mMinAngle_Cumulative);
    }
    return val/dif;
  }
}