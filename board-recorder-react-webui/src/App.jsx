import { RouterProvider, createBrowserRouter } from "react-router-dom";
import { QueryClientProvider } from "@tanstack/react-query";

import RecordingsPage from "./pages/Recordings.jsx";
import Root from "./components/Root/Root.jsx";
import IntroPage from "./pages/Intro.jsx";
import NotFoundBoundary from "./components/ErrorPages/NotFoundBoundary.jsx";

import { queryClient } from "./query_utils/db.js";
import { getRosBridgeAddress } from "./query_utils/auth.js";

function App() {
  const router = createBrowserRouter([
    {
      path: "/",
      loader: getRosBridgeAddress,
      id: "root",
      element: <Root />,
      errorElement: <NotFoundBoundary />,
      children: [
        { index: true, element: <IntroPage /> },
        { path: "intro", element: <IntroPage /> },
        {
          path: "recordings",
          element: <RecordingsPage />,
        },
      ],
    },
  ]);

  return (
    <QueryClientProvider client={queryClient}>
      <RouterProvider router={router} />
    </QueryClientProvider>
  );
}

export default App;
