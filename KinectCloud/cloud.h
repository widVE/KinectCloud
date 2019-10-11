#pragma once

#include "util.h"

//              .
//               					
//              |					
//     .               /				
//      \       I     				
//                  /
//        \  ,g88R_
//          d888(`  ).                   _
// -  --==  888(     ).=--           .+(`  )`.
//)         Y8P(       '`.          :(   .    )
//        .+(`(      .   )     .--  `.  (    ) )
//       ((    (..__.:'-'   .=(   )   ` _`  ) )
//`.     `(       ) )       (   .  )     (   )  ._
//  )      ` __.:'   )     (   (   ))     `-'.:(`  )
//)  )  ( )       --'       `- __.'         :(      ))
//.-'  (_.'          .')                    `(    )  ))
//                  (_  )                     ` __.:'
//                                        	
//--..,___.--,--'`,---..-.--+--.,,-,,..._.--..-._.-a:f--.

namespace kinectCloud {
#ifdef KINECTCLOUD_EXPERIMENTAL
	//call cloudcompare to transform
	void merge(std::string cloudComparePath, std::string reference, std::string addition, std::string outpath, std::string transPath = "") {
		std::cout << "merging " << addition << "and " << reference << " into " << outpath << "\n";

		std::string invoke =
			"start powershell.exe \"& '" + cloudComparePath + "'"
			" -SILENT -LOG_FILE log.txt -AUTO_SAVE OFF"
			" -O '" + addition + "'" + (transPath.empty() ? "" : " -APPLY_TRANS '" + transPath + "'") + ""
			" -O '" + reference + "'"
			" -ICP -OVERLAP 80 -MERGE_CLOUDS -SS SPATIAL 0.1"
			" -SAVE_CLOUDS FILE '" + outpath + "'\"";

		exec(invoke.c_str());
	}
#endif
}