vtk_add_test_cxx(vtkCommonMathCxxTests tests
  NO_DATA NO_VALID NO_OUTPUT
  TestAmoebaMinimizer.cxx
  TestMatrix3x3.cxx
  TestPolynomialSolversUnivariate.cxx
  TestQuaternion.cxx
  TestQuaternionInterpolator.cxx
  TestReservoirSampler.cxx
  UnitTestFFT.cxx
  )
vtk_test_cxx_executable(vtkCommonMathCxxTests tests)
