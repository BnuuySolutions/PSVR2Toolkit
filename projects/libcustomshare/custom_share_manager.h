#pragma once

class CustomShareManager {
public:
  static void createSingleton();
  static CustomShareManager *getSingleton();

private:
  static CustomShareManager *m_pInstance;
  static bool m_initialized;

  void initialize();
};
