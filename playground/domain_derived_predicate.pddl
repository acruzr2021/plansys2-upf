(define (domain warehouse-logistics)
  (:requirements :strips :typing :fluents :derived-predicates)
  
  (:types
    robot location package - object
    warehouse depot - location
  )
  
  (:predicates
    (at-robot ?r - robot ?l - location)
    (at-package ?p - package ?l - location)
    (carrying ?r - robot ?p - package)
    (connected ?l1 - location ?l2 - location)
    (empty ?r - robot)
    
    ;; Predicados derivados (se calculan automáticamente)
    (low-battery ?r - robot)
    (overloaded ?r - robot)
    (location-busy ?l - location)
    (can-pick-more ?r - robot)
  )
  
  (:functions
    (battery-level ?r - robot)
    (battery-consumption ?l1 ?l2 - location)
    (load-weight ?r - robot)
    (max-load ?r - robot)
    (package-weight ?p - package)
  )
  
  ;; Predicado derivado: batería baja (menos del 20%)
  (:derived (low-battery ?r - robot)
    (< (battery-level ?r) 20)
  )
  
  ;; Predicado derivado: robot sobrecargado
  (:derived (overloaded ?r - robot)
    (> (load-weight ?r) (max-load ?r))
  )
  
  ;; Predicado derivado: puede cargar más paquetes
  (:derived (can-pick-more ?r - robot)
    (and
      (not (overloaded ?r))
      (< (load-weight ?r) (max-load ?r))
    )
  )
  
  ;; Predicado derivado: ubicación ocupada (hay algún robot allí)
  (:derived (location-busy ?l - location)
    (exists (?r - robot)
      (at-robot ?r ?l)
    )
  )
  
  ;; Acción: mover el robot
  (:action move
    :parameters (?r - robot ?from ?to - location)
    :precondition (and
      (at-robot ?r ?from)
      (connected ?from ?to)
      (not (low-battery ?r))  ;; Usa predicado derivado
      (not (overloaded ?r))   ;; Usa predicado derivado
      (>= (battery-level ?r) (battery-consumption ?from ?to))
    )
    :effect (and
      (not (at-robot ?r ?from))
      (at-robot ?r ?to)
      (decrease (battery-level ?r) (battery-consumption ?from ?to))
    )
  )
  
  ;; Acción: recoger paquete
  (:action pick
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (at-robot ?r ?l)
      (at-package ?p ?l)
      (can-pick-more ?r)  ;; Usa predicado derivado
      (<= (+ (load-weight ?r) (package-weight ?p)) (max-load ?r))
    )
    :effect (and
      (not (at-package ?p ?l))
      (carrying ?r ?p)
      (not (empty ?r))
      (increase (load-weight ?r) (package-weight ?p))
    )
  )
  
  ;; Acción: dejar paquete
  (:action drop
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (at-robot ?r ?l)
      (carrying ?r ?p)
    )
    :effect (and
      (not (carrying ?r ?p))
      (at-package ?p ?l)
      (decrease (load-weight ?r) (package-weight ?p))
    )
  )
  
  ;; Acción: recargar batería
  (:action recharge
    :parameters (?r - robot ?l - warehouse)
    :precondition (and
      (at-robot ?r ?l)
      (low-battery ?r)  ;; Usa predicado derivado
    )
    :effect (and
      (assign (battery-level ?r) 100)
    )
  )
  
  ;; Acción: descargar peso (vaciar robot)
  (:action unload-all
    :parameters (?r - robot ?l - depot)
    :precondition (and
      (at-robot ?r ?l)
      (overloaded ?r)  ;; Usa predicado derivado para forzar descarga
    )
    :effect (and
      (assign (load-weight ?r) 0)
      (empty ?r)
      (forall (?p - package)
        (when (carrying ?r ?p)
          (and
            (not (carrying ?r ?p))
            (at-package ?p ?l)
          )
        )
      )
    )
  )
)