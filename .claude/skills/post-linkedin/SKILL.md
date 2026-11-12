---
name: post-linkedin
description: Brouillon de post LinkedIn technique sur une à deux semaines d'avancement du projet MyCobot R6A (apprentissages, résultats majeurs, setup, positionnement face à l'état de l'art). À invoquer toutes les une à deux semaines, quand `etat_session.sh` le rappelle, ou après un résultat marquant. Ne publie jamais.
---

# Post LinkedIn — avancement du projet

Produit un **brouillon**, jamais une publication : poster engage l'équipe, et
c'est toujours un humain qui le fait. Les brouillons vivent hors du dépôt,
dans `~/dream_article/communication/linkedin/` (ils peuvent citer des chiffres
non encore validés pour l'article).

## Réglages

Lus dans `~/dream_article/communication/linkedin/reglages.yaml` (créé au
premier passage avec ces valeurs par défaut) :

```yaml
auteur: encadrant          # ou: osama
langue: fr                 # ou: en
embargo: actif             # « levé » une fois l'article soumis ou accepté
crediter_osama: oui        # nommer le stagiaire quand son travail est en jeu
cadence_jours: 14          # rappel au démarrage de session au-delà
```

## Déroulé

1. **Fenêtre.** Du dernier brouillon (`ls ~/dream_article/communication/linkedin/20*.md`)
   à aujourd'hui, 14 jours par défaut, 7 au minimum.
2. **Y a-t-il matière ?** Lire `git log --since`, les entrées datées de
   `SESSION_RESUME.md` et de « Ce qu'on a appris » (`docs/ARTICLE_DREAM.md`)
   dans la fenêtre. Il faut au moins **un** de :
   - un chiffre passé `tenu` au registre ;
   - un apprentissage technique qui servirait à d'autres (une dérive mesurée et
     expliquée, une hypothèse réfutée, un piège d'outil) ;
   - un jalon visible (jeu de données figé, banc réel, démonstration).

   Sinon, dire « pas assez de matière depuis le JJ/MM » et s'arrêter : un post
   sans substance coûte plus qu'il ne rapporte.
3. **Rédiger** : agent `communicant-linkedin`, avec la fenêtre et les réglages.
4. **Vérifier les chiffres** : agent `metrologue` sur la section
   « Vérification » du brouillon. Tout chiffre qu'il juge non soutenu sort du
   post (il passe dans « Écarté »).
5. **Présenter** à l'utilisateur : le post, l'accroche de rechange, le visuel,
   et les points de « Écarté » à trancher. Ne rien publier.

## Garde-fous

- **Embargo** : tant que `embargo: actif`, pas de résultat central de l'article
  (points de vue réels tenus à l'écart) ni de méthode détaillée qui le produit.
- **Pas de « premier » ni d' « inédit »** : voir `training/dream/article/biblio.md`.
- **Aucune donnée de poste** (IP, chemins, noms de machine) ni de personne hors
  de l'équipe.
- Un post par fenêtre ; ne pas répéter le précédent.
