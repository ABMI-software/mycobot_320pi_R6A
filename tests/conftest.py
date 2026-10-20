"""Deport d'outil explicite pour la suite de tests.

`scripts/tool_offset.json` n'a jamais ete commite (verifie sur tout
l'historique). Les tests geometriques en dependaient donc d'un fichier local
absent de tout autre poste : la collecte echouait sur cinq fichiers, et le
test de calibration du deport n'avait plus aucune chance d'etre rejoue.

La valeur ci-dessous est celle que les tests COMMITES encodent, pas une
mesure faite ici : `test_la_reference_de_l_outil_est_bien_le_bout_des_doigts`
exige `pointe(q_contact).Z = 0 +/- 1 mm`, ce que ce vecteur donne a +0,04 mm.
Elle vient de `docs/PICK_AND_PLACE_BOUCLE_FERMEE.md` 5.3 : longueur 110,4 mm,
lateraux -8,9 et +11,9, axe `-X` de la bride.

NE PAS l'utiliser sur le robot. Le bloc de recalage du 09/09 dans
`pick_fsm.py` decrit le meme deport raccourci de 17,33 mm, ce qui donnerait
+17,1 mm au meme point de contact. Les deux ne peuvent pas etre vraies a la
fois et seule une mesure physique tranche. Tant qu'elle n'est pas faite, ce
fichier fige la geometrie CONTRE LAQUELLE LES TESTS ONT ETE ECRITS, afin que
les 180 autres assertions redeviennent executables.
"""
import os

os.environ.setdefault('MYCOBOT_TOOL_OFFSET_MM', '-109.395,-8.9,11.9')
