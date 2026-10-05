// ============================================================================
//  nqg_apartment.hpp  -  Facade di compatibilita'
//  ---------------------------------------------------------------------------
//  FIX 2025: questo header conteneva per errore una copia IDENTICA di
//  nqg_earth_environment.hpp, con lo STESSO include guard
//  (NQG_EARTH_ENVIRONMENT_HPP). Di conseguenza il suo contenuto non era mai
//  attivo (il guard scattava sul primo include). Inoltre la vera geometria
//  dell'appartamento vive in nqg::apartment dentro nqg_cleanroom_engine.hpp.
//
//  Questo header e' ora una facade sottile che riesporta l'ambiente terrestre,
//  cosi' chi includeva nqg_apartment.hpp per errore continua a compilare.
// ============================================================================
#ifndef NQG_APARTMENT_HPP
#define NQG_APARTMENT_HPP

#include "nqg_earth_environment.hpp"

// La geometria della stanza (RoomGeometry, CapsuleCollider, AABB, ...) e'
// definita nel namespace nqg::apartment all'interno di
// nqg_cleanroom_engine.hpp. Per usarla:
//   #include "nqg_cleanroom_engine.hpp"

#endif // NQG_APARTMENT_HPP